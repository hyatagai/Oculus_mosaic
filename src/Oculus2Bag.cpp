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

  public:
    Oculus2Image();
    ~Oculus2Image();
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
            if (nowdeg > th) {
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
            if (nowdeg > th) {
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
            int im_ind = (sonar_image.height - h - 1) * sonar_image.step + (sonar_image.width - w - 1);
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

void oculus2imagebag(std::string input, std::string output)
{
    Oculus2Image o2i;
    rosbag::Bag bag_read;
    rosbag::Bag bag_write;
    bag_read.open(input, rosbag::bagmode::Read);
    bag_write.open(output, rosbag::bagmode::Write);

    std::vector<std::string> topics;
    topics.push_back(std::string("/blueprint_oculus"));

    rosbag::View view(bag_read, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        blueprint_oculus::BlueprintOculus8bit::Ptr oculus_data = m.instantiate<blueprint_oculus::BlueprintOculus8bit>();
        if (oculus_data != NULL)
        {
            sensor_msgs::Image sonar_image_cartesian = o2i.calc_cartesian_image(oculus_data);
            sensor_msgs::Image sonar_image_polar = o2i.calc_polar_image(oculus_data);
            bag_write.write("/blueprint_oculus/sonar_image/cartesian", sonar_image_cartesian.header.stamp, sonar_image_cartesian);
            bag_write.write("/blueprint_oculus/sonar_image/polar", sonar_image_polar.header.stamp, sonar_image_polar);
            bag_write.write("/blueprint_oculus", oculus_data->header.stamp, oculus_data);
        }
    }

    bag_read.close();
    bag_write.close();

}

int main(int argc, char *argv[])
{
    std::string inp1 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_121755.bag";
    std::string out1 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_121755_image.bag";
    oculus2imagebag(inp1, out1);
    std::string inp2 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_132950.bag";
    std::string out2 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_132950_image.bag";
    oculus2imagebag(inp2, out2);
    std::string inp3 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_133717.bag";
    std::string out3 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_133717_image.bag";
    oculus2imagebag(inp3, out3);
    std::string inp4 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_134454.bag";
    std::string out4 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_134454_image.bag";
    oculus2imagebag(inp4, out4);
    std::string inp5 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_135226.bag";
    std::string out5 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_135226_image.bag";
    oculus2imagebag(inp5, out5);
    std::string inp6 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_135956.bag";
    std::string out6 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_135956_image.bag";
    oculus2imagebag(inp6, out6);
    std::string inp7 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_140723.bag";
    std::string out7 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_140723_image.bag";
    oculus2imagebag(inp7, out7);
    std::string inp8 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_141447.bag";
    std::string out8 = "/media/horimoto/horimoto1/2018_6_神戸ラグーン/解析後データ/rosbag/20180618_141447_image.bag";
    oculus2imagebag(inp8, out8);

}