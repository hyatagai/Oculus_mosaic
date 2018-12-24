#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag

def bag_merge(bag_new_name, bag_name_list):
    bag_new = rosbag.Bag(bag_new_name, "w")
    for bag_name in bag_name_list:
        bag_read = rosbag.Bag(bag_name)
        for topic, msg, t in bag_read.read_messages():
            bag_new.write(topic, msg, t)
        bag_read.close()
    bag_new.close()


if __name__ == "__main__":
    for i in range(31, 33):
        bag_name1 = "/media/horimoto/horimoto2/ooarai_20180209/rosbag/merged/bag" + str(i) + "_sonar_camera.bag"
        bag_name2 = "/media/horimoto/horimoto2/ooarai_20180209/rosbag/merged/bag" + str(i) + "_sonar_camera_gamma.bag"
        bag_new_name = "/media/horimoto/horimoto2/ooarai_20180209/rosbag/merged/bag" + str(i) + "_scg_merged.bag"
        bag_merge(bag_new_name, [bag_name1, bag_name2])

