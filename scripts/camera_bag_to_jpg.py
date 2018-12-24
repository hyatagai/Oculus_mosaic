#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
import rospy
import cv2
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import os


def bag_to_jpg(bag_pass, img_pass, topics):
    bag_read = rosbag.Bag(bag_pass)
    bridge = CvBridge()
    print("read", bag_pass)
    print("write to ", img_pass)
    for _, msg, t in bag_read.read_messages(
            topics=topics):
        cv_image = bridge.imgmsg_to_cv2(msg, "mono8")
        new_filename = topics.split("/")[-1] + "_" + str(t.secs) + "_" + str(t.nsecs / 1000000) + ".jpg"
        new_filepath = img_pass + "/" + new_filename
        print("write to ", new_filepath)
        cv2.imwrite(new_filepath, cv_image)
        # time.sleep(0.1)
    bag_read.close()


if __name__ == "__main__":
    topic_cartesian = "/blueprint_oculus/sonar_image/cartesian"
    topic_polar = "/blueprint_oculus/sonar_image/polar"
    bag_pass = ""/home/umigame/seatec/20181004_シーテック/"
    file_name_list = ["2018-10-04-11-20-29_8bit_oculus_time_corrected_with_"]
    suffix = "_image.bag"
    for file_name in file_name_list:
        bag_pass1 = bag_pass + file_name + suffix
        img_dir_pass = bag_pass + file_name
        os.mkdir(img_dir_pass)
        bag_to_jpg(bag_pass1, img_dir_pass, topic_cartesian)
        bag_to_jpg(bag_pass1, img_dir_pass, topic_polar)
