#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rosbag
import roslib
import rospy
import cv2
roslib.load_manifest('sensor_msgs')

topic_name = 'omni_cam/image_raw'


def CreateVideoBag(videopath, bagname):
    '''Creates a bag file with a video file'''
    bag = rosbag.Bag(bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = 30
    ret = True
    frame_id = 0
    while (ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(1518153732 + float(frame_id) / prop_fps)
        frame_id += 1
        image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(topic_name, image, stamp)
        if frame_id%1800==0:
            print frame_id/1800, "min"
    cap.release()
    bag.close()


if __name__ == "__main__":
    filepath = "/home/horimoto/Documents/ooarai_videos/set1_all_inverted_360.mp4"
    bagname = "ooarai20180209_set1_720.bag"
    CreateVideoBag(filepath, bagname)
