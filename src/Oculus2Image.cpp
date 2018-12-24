#pragma once

#include <string>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "blueprint_oculus/BlueprintOculus.h"
#include "blueprint_oculus/BlueprintOculus8bit.h"
#include "blueprint_oculus/BlueprintOculusConfig.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

class Oculus2Image
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/blueprint_oculus", 1000, &Oculus2Image::callback, this);
    ros::Publisher pub_c = nh.advertise<sensor_msgs::Image>("/blueprint_oculus/sonar_image/cartesian", 100);
    ros::Publisher pub_p = nh.advertise<sensor_msgs::Image>("/blueprint_oculus/sonar_image/polar", 100);

  public:
    Oculus2Image();
    ~Oculus2Image();
    void callback(const blueprint_oculus::BlueprintOculus8bit::ConstPtr &msg);
    sensor_msgs::Image calc_cartesian_image(const blueprint_oculus::BlueprintOculus8bit::ConstPtr &msg);
    sensor_msgs::Image calc_polar_image(const blueprint_oculus::BlueprintOculus8bit::ConstPtr &msg);
    float deg2rad(float num);
    float rad2deg(float num);
};

Oculus2Image::Oculus2Image()
{
}

Oculus2Image::~Oculus2Image()
{
}

void Oculus2Image::callback(const blueprint_oculus::BlueprintOculus8bit::ConstPtr &msg)
{
    sensor_msgs::Image sonar_image_polar = calc_polar_image(msg);
    sensor_msgs::Image sonar_image_cartesian = calc_cartesian_image(msg);
    pub_c.publish(sonar_image_cartesian);
    pub_p.publish(sonar_image_polar);
}

sensor_msgs::Image Oculus2Image::calc_cartesian_image(const blueprint_oculus::BlueprintOculus8bit::ConstPtr &msg)
{
    sensor_msgs::Image sonar_image;
    sonar_image.header.frame_id = msg->header.frame_id;
    sonar_image.header.stamp = msg->header.stamp;
    sonar_image.encoding = sensor_msgs::image_encodings::MONO8;
    sonar_image.is_bigendian = 0;
    sonar_image.height = msg->n_ranges;
    float hor_aperture_half = deg2rad((float)(msg->bearing[0]) / 100.0);
    float fan_width = std::ceil(std::sin(std::abs(hor_aperture_half)) * sonar_image.height) * 2;
    sonar_image.width = (int)fan_width;
    sonar_image.step = sonar_image.width;
    sonar_image.is_bigendian = 0;
    sonar_image.data.resize(sonar_image.step * sonar_image.height);

    int bearing_min = msg->bearing.front();
    int bearing_max = msg->bearing.back();
    int length = bearing_max - bearing_min + 1;

    // make bearing table. Index is [0.01 deg]
    std::vector<int> bearing_table(length);

    for (int i = 0, bearing_ind = 0; i < length; i++)
    {
        float nowdeg = i + bearing_min;
        if (bearing_ind != msg->n_beams - 1)
        {
            float th = (float)(msg->bearing[bearing_ind] + msg->bearing[bearing_ind + 1]) / 2.;
            while (nowdeg > th) {
                bearing_ind += 1;
                th = (float)(msg->bearing[bearing_ind] + msg->bearing[bearing_ind + 1]) / 2.;
            }
        }
        bearing_table[i] = bearing_ind;
    }

    for (int h = 0; h < sonar_image.height; h++)
    {
        for (int w = 0; w < sonar_image.width; w++)
        {
            float posx = sonar_image.height - (float)h;
            float posy = sonar_image.width / 2. - (float)w;
            float dist = std::sqrt(posx * posx + posy * posy);
            float theta = rad2deg(std::atan2(posy, posx));

            unsigned short intensity = 0;
            if (dist < sonar_image.height && (float)bearing_min / 100. < theta && theta < (float)bearing_max / 100.)
            {
                int dist_ind = int(std::floor(dist));
                int theta_rounded = int(theta * 100) - bearing_min;
                int theta_ind = bearing_table[theta_rounded];
                int oculus_ind = dist_ind * msg->n_beams + theta_ind;
                intensity = msg->data[oculus_ind];
            }
            int im_ind = h * sonar_image.step + w;
            sonar_image.data[im_ind] = (unsigned char)(intensity);
        }
    }
    return sonar_image;
}

sensor_msgs::Image Oculus2Image::calc_polar_image(const blueprint_oculus::BlueprintOculus8bit::ConstPtr &msg)
{
    sensor_msgs::Image sonar_image;
    sonar_image.header.frame_id = msg->header.frame_id;
    sonar_image.header.stamp = msg->header.stamp;
    sonar_image.encoding = sensor_msgs::image_encodings::MONO8;
    sonar_image.is_bigendian = 0;
    sonar_image.height = msg->n_ranges;
    int hor_aperture_half = std::abs(msg->bearing[0] / 10); //0.1 degree
    sonar_image.width = hor_aperture_half * 2 + 1;
    sonar_image.step = sonar_image.width;
    sonar_image.is_bigendian = 0;
    sonar_image.data.resize(sonar_image.step * sonar_image.height);

    int bearing_min = int(msg->bearing.front() / 10);
    int bearing_max = int(msg->bearing.back() / 10);
    int length = bearing_max - bearing_min + 1;

    // make bearing table. Index is [0.1 deg]
    std::vector<int> bearing_table(length);
    for (int i = 0, bearing_ind = 0; i < length; i++)
    {
        float nowdeg = i + bearing_min;
        if (bearing_ind != msg->n_beams - 1)
        {
            float th = (float)(msg->bearing[bearing_ind] / 10 + msg->bearing[bearing_ind + 1] / 10) / 2.;
            while (nowdeg > th) {
                bearing_ind += 1;
                th = (float)(msg->bearing[bearing_ind] / 10 + msg->bearing[bearing_ind + 1] / 10) / 2.;
            }
        }
        bearing_table[i] = bearing_ind;
    }

    for (int h = 0; h < sonar_image.height; h++)
    {
        for (int w = 0; w < sonar_image.width; w++)
        {
            unsigned short intensity;
            int oculus_ind = h * msg->n_beams + bearing_table[w];
            intensity = msg->data[oculus_ind];
            int im_ind = h * sonar_image.step + w;
            sonar_image.data[im_ind] = (unsigned char)(intensity);
        }
    }

    return sonar_image;
}

float Oculus2Image::deg2rad(float num)
{
    return num * 3.14159265358979 / 180.;
}

float Oculus2Image::rad2deg(float num)
{
    return num * 180. / 3.14159265358979;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "oculus2image");
    ROS_INFO("START Oculus to image conversion");

    Oculus2Image o2i;
    ros::spin();
}