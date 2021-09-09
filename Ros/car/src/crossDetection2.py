#!/usr/bin/env python
# use encoding: utf-8
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import  Int32

flag  = Int32()
left_obs_prev = -90
right_obs_prev = 90
count = 0
gear = Int32()

def gear_callback(msg):
    global gear
    gear = msg.data


def laserCallback(msg):
    global flag, left_obs_prev, right_obs_prev, count,gear
    if count < 2:
        count = count + 1
        return
    else:
        count = 0
    flag = 0
    flag_laser = rospy.Publisher("flag_cross",Int32,queue_size=1)

    left_angle = 60    #range of scan angle
    right_angle = -10

    left_obs = -90
    right_obs = 90
    for angle in range(right_angle, left_angle):
        i = (angle+180)*4
        if msg.ranges[i] < 1.5 and msg.ranges[i+1] < 1.5 and msg.ranges[i+2] < 1.5:
 #       if i < 720:
            if msg.ranges[i] > 0.85:#0.8
                continue
            if angle < right_obs:
                right_obs = angle
            if angle > left_obs:
                left_obs = angle
    #print ("left_angle: %f, right_angle: %f" % (left_obs, right_obs))
    if (left_obs < -10):
        #print ("No obs")
        flag = 1
    #elif left_obs < 15 and abs(left_obs-left_obs_prev) < 1:
     #   print ("Obs removed")
      #  flag = 1
    #else:
    	#print ("obs detected")
    left_obs_prev = left_obs

    #if gear == 3:
    flag_laser.publish(flag)
    print ("pds flag: ", flag)

if __name__ == "__main__":
    try:
        print("crossDetection2 open")
        rospy.init_node('laser_detection', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, laserCallback)
        rospy.Subscriber("/auto_driver/send/gear",Int32,gear_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
