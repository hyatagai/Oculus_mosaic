#pragma once

#include <string>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "blueprint_oculus/BlueprintOculus.h"
#include "blueprint_oculus/BlueprintOculusConfig.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

sensor_msgs::Image gamma2Image(sensor_msgs::Image::Ptr &image, float gamma)
{
    sensor_msgs::Image ret;
    ret.header.frame_id = image->header.frame_id;
    ret.header.stamp = image->header.stamp;
    ret.height = image->height;
    ret.width = image->width;
    ret.encoding = image->encoding;
    ret.is_bigendian = image->is_bigendian;
    ret.step = image->step;
    ret.data.resize(ret.step * ret.height);
    for (int i = 0; i < ret.height; i++)
    {
        for (int j = 0; j < ret.width; j++)
        {
            int ind = i * ret.step + j * 2;
            float intensity = image->data[ind] + image->data[ind+1] * 256;
            float new_intensity = std::pow(intensity / 65535., gamma) * 65535.;
            int new_intensity_int = (int) (new_intensity + 0.5);
            ret.data[ind] = new_intensity_int % 256;
            ret.data[ind+1] = new_intensity_int / 256;
        }
    }
    return ret;
}


void addGammaImage2bag(std::string input, std::string output)
{
    float gamma = 150./ 256.;   //from sdk pdf
    rosbag::Bag bag_read;
    rosbag::Bag bag_write;
    bag_read.open(input, rosbag::bagmode::Read);
    bag_write.open(output, rosbag::bagmode::Write);

    std::vector<std::string> topics;
    topics.push_back(std::string("/blueprint_oculus/sonar_image"));
    // topics.push_back(std::string("/blueprint_oculus/sonar_image_rect"));

    rosbag::View view(bag_read, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::Ptr sonar_image = m.instantiate<sensor_msgs::Image>();
        if (sonar_image != NULL)
        {
            sensor_msgs::Image gamma_sonar_image = gamma2Image(sonar_image, gamma);
            bag_write.write("/blueprint_oculus/sonar_image/gamma", sonar_image->header.stamp, gamma_sonar_image);
        }
        // sensor_msgs::Image::Ptr sonar_image_rect = m.instantiate<sensor_msgs::Image>();
        // if (sonar_image_rect != NULL)
        // {
        //     sensor_msgs::Image gamma_sonar_image_rect = gamma2Image(sonar_image_rect, gamma);
        //     bag_write.write("/blueprint_oculus/sonar_image_rect/gamma", sonar_image_rect->header.stamp, gamma_sonar_image_rect);
        // }
    }

    bag_read.close();
    bag_write.close();

}

int main(int argc, char *argv[])
{
    std::string header = "/media/horimoto/horimoto2/ooarai_20180209/rosbag/merged/bag";
    for (int i = 19; i <= 32; i++)
    {
        std::string input_name = header + std::to_string(i) + "_sonar_camera.bag";
        std::string output_name = header + std::to_string(i) + "_sonar_camera_gamma.bag";
        std::cout << "processing: " << input_name << std::endl;
        addGammaImage2bag(input_name, output_name);
    }
}