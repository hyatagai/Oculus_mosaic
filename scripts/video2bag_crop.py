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


def CreateVideoBag(videopath, bagname, start_time_unix):
    '''Creates a bag file with a video file'''
    bag = rosbag.Bag(bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = 29.97
    ret = True
    frame_id = 0
    upper = 10.
    lower = -20.
    width = 130.
    fov = 235.

    while (ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(start_time_unix + float(frame_id) / prop_fps)
        frame_id += 1
        row = len(frame[0, :, 0])
        h_min = int(row / 2. - row * upper / fov)
        h_max = int(row / 2. - row * lower / fov)
        w_m = int((1 - width / fov) * row / 2.)
        new_frame = frame[h_min:h_max, w_m:-w_m, :]
        image = cb.cv2_to_imgmsg(new_frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(topic_name, image, stamp)
        if frame_id%1800==0:
            print frame_id/1800, "min"
    cap.release()
    bag.close()



if __name__ == "__main__":
    set1_start_time_unix = 1518153733.5
    set2_start_time_unix = 1518158945.0
    set3_start_time_unix = 1518162212.0
    # filepath = "/home/horimoto/Documents/ooarai_videos/set2_all_inverted_720.mp4"
    # bagname = "ooarai20180209_set2_720_cropped.bag"
    # CreateVideoBag(filepath, bagname, set2_start_time_unix)
    filepath = "/home/horimoto/Documents/ooarai_videos/set3_all_inverted_720.mp4"
    bagname = "ooarai20180209_set3_720_cropped.bag"
    CreateVideoBag(filepath, bagname, set3_start_time_unix)
    # filepath = "/home/horimoto/Documents/ooarai_videos/set1_all_inverted_1440.mp4"
    # bagname = "ooarai20180209_set1_1440_cropped.bag"
    # CreateVideoBag(filepath, bagname)
    # filepath = "/home/horimoto/Documents/ooarai_videos/set2_all_inverted_1440.mp4"
    # bagname = "ooarai20180209_set2_1440_cropped.bag"
    # CreateVideoBag(filepath, bagname)
    # filepath = "/home/horimoto/Documents/ooarai_videos/set3_all_inverted_1440.mp4"
    # bagname = "ooarai20180209_set3_1440_cropped.bag"
    # CreateVideoBag(filepath, bagname)
