#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import numpy as np
# from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
import cv2

imSize = (520, 520)

""" sub topics """
topic_scan = "/scan"
def callback_scan(scan):
    img = np.zeros(imSize, dtype=np.uint8)
    ranges = np.array(scan.ranges)  # [1440,]
    ranges[ranges>2.5] = 0
    angle = np.arange(len(scan.ranges)) * scan.angle_increment
    px = ( np.sin(angle) * ranges * 100 ).astype(np.int32) + 260
    py = ( np.cos(angle) * ranges * 100 ).astype(np.int32) + 260
    img[px, py] = 255
    cv2.imshow('scan', img)
    cv2.waitKey(5)


def main():
    #--- node init
    rospy.init_node('lidar_port', anonymous=True)
    print("[lidar_port]: Init")

    #--- subscriber topic
    rospy.Subscriber(topic_scan, LaserScan, callback_scan)

    rospy.spin()

if __name__ == '__main__':
    main()


