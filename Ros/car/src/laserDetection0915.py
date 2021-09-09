#!/usr/bin/env python
# use encoding: utf-8
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import  Int32

debug = 0
visualize = debug
count = 0
cross_flag = False
board_flag = True
roadWidth = 0.45
y_left = 0.6
y_right = -0.6
point_num = 80 #80
fit_thresold = 0.1
b_thresold = 1.0
flag  = Int32()
flag_laser = Int32()


def laserCallback(msg):
    global debug, visualize, count, cross_flag, board_flag, roadWidth, y_left, y_right, point_num, fit_thresold, b_thresold
    global flag,flag_laser

    cmdPub = rospy.Publisher('laser_direction', Twist, queue_size=1)  # 话题名记得改过来
    #flag_laserPub = rospy.Publisher("flag_laser",Int32,queue_size=1)
    cam_cmd = Twist()
    if debug:
        if count < 10:
            print("count: ", count)
            count = count + 1
            return
        elif count == 10:
            count = count + 1
        elif count > 10:
            return
    #print("Start processing")
    if board_flag:
        if visualize:
            plt.close()
        left_angle_max = 90
        left_angle_min = 20
        right_angle_max = 90
        right_angle_min = 20

        pointcloud_left_x = []
        pointcloud_left_y = []
        range_left = []
        pointcloud_right_x = []
        pointcloud_right_y = []
        range_right = []
        for i in range(1440):
            if (msg.ranges[i] > 1.1):#1.1
                continue
            if (i > (720 - 4*right_angle_max)) & (i < (720 - 4*right_angle_min)):
                angle = math.pi / 720 * i - math.pi
                y_tmp = msg.ranges[i] * math.sin(angle)
                if y_tmp < y_left:
                    x_tmp = msg.ranges[i] * math.cos(angle)
                    pointcloud_right_x.append(x_tmp)
                    pointcloud_right_y.append(y_tmp)
                    range_right.append(msg.ranges[i])
            if (i < (720 + 4*left_angle_max)) & (i > (720 + 4*left_angle_min)):
                angle = math.pi / 720 * i - math.pi
                y_tmp = msg.ranges[i] * math.sin(angle)
                if y_tmp > y_right:
                    x_tmp = msg.ranges[i] * math.cos(angle)
                    pointcloud_left_x.append(x_tmp)
                    pointcloud_left_y.append(y_tmp)
                    range_left.append(msg.ranges[i])
	
        print("PointcloudLeft size: ", len(pointcloud_left_x))
        print("PointcloudRight size: ", len(pointcloud_right_x))
	if (len(pointcloud_left_x)<15) or (len(pointcloud_right_x)<15):#15
	    cam_cmd.angular.z = 0
	    cam_cmd.angular.x = 0
	    cmdPub.publish(cam_cmd)
	    return
        if (len(pointcloud_left_x) < point_num) & (len(pointcloud_right_x) < point_num):
            #print("cannot detect enough point")
            # 加入是否检测成功的指令
            flag_laser = 0
            cam_cmd.angular.z = 0
            cam_cmd.angular.x = flag_laser
            cmdPub.publish(cam_cmd)
            return
	#if (len(pointcloud_left_x)-len(pointcloud_right_x))>180:
	    #return
        flag = 0
        fit_right = 0
        k_right = 1
        b_right = 1
        error_right = 1
        if len(pointcloud_right_x) >= point_num:
            coff_right, status_right = np.polynomial.polynomial.Polynomial.fit(
                pointcloud_right_x, pointcloud_right_y, 1, full=True)
            k_right = coff_right.convert().coef[1]
            b_right = coff_right.convert().coef[0]
            error_right = status_right[0]
            #print("fit the right board, k is %f, b is %f, error is %f" %
                  #(k_right, b_right, error_right))
            if (debug):
                plt.plot(pointcloud_right_x, pointcloud_right_y)
                x_min = min(pointcloud_right_x)
                x_max = max(pointcloud_right_x)
                x_plot = np.linspace(x_min, x_max, 50)
                y_plot = k_right * x_plot + b_right
                plt.plot(x_plot, y_plot)

            if (error_right < fit_thresold) & (abs(b_right) < b_thresold) & (abs(k_right) < 1):
                #print("the right borad can fit properly")
                fit_right = 1
                flag = 1

        k_left = 1
        b_left = 1
        error_left = 1
        fit_left = 0
        if (len(pointcloud_left_x) > point_num) & (not fit_right):
            coff_left, status_left = np.polynomial.polynomial.Polynomial.fit(
                pointcloud_left_x, pointcloud_left_y, 1, full=True)
            k_left = coff_left.convert().coef[1]
            b_left = coff_left.convert().coef[0]
            error_left = status_left[0]
            #print("fit the left board, k is %f, b is %f, error is %f" %
                  #(k_left, b_left, error_left))

            if (debug):
                plt.plot(pointcloud_left_x, pointcloud_left_y)
                x_min = min(pointcloud_left_x)
                x_max = max(pointcloud_left_x)
                x_plot = np.linspace(x_min, x_max, 50)
                y_plot = k_left * x_plot + b_left
                plt.plot(x_plot, y_plot)

            if (error_left < fit_thresold) & (abs(b_left) < b_thresold) and (abs(k_left) < 1):
             #   print("the left borad can fit properly")
                fit_left = 1
                flag = -1
	if fit_left==1:
	    if abs(b_left)>0.65:#0.65
	    	flag_laser = 0
            	cam_cmd.angular.z = 0
            	cam_cmd.angular.x = flag_laser
            	cmdPub.publish(cam_cmd)
	    
	    	return
	if fit_right==1:
	    if abs(b_right)>0.65:#0.65
	    	flag_laser = 0
            	cam_cmd.angular.z = 0
            	cam_cmd.angular.x = flag_laser
            	cmdPub.publish(cam_cmd)
	    	return
	    
        if (not fit_left) & (not fit_right):
            #print("both side can not fit properly")
	    if (len(pointcloud_left_x) > point_num) & (len(pointcloud_right_x) > point_num):
            	flag_laser = 2
	    else:
		flag_laser = 2
            cam_cmd.angular.z = 0
            cam_cmd.angular.x = flag_laser
            cmdPub.publish(cam_cmd)
	    return

        pose_x = 1.0
	#pos_x = 2.5
        x_center_single = 0
        y_center_single = 0
        if flag == 1:
            pose_y = k_right * pose_x + b_right
            x_center_single = pose_x + roadWidth / \
                ((1 + k_right**2) ** 0.5) * (-k_right)
            y_center_single = pose_y + roadWidth / ((1 + k_right**2)**0.5)
            
            y_center_single = (pose_x-x_center_single) * \
                k_right+y_center_single
            x_center_single = pose_x
        else:
            pose_y = k_left * pose_x + b_left
            x_center_single = pose_x - roadWidth / \
                ((1 + k_left**2) ** 0.5) * (-k_left)
            y_center_single = pose_y - roadWidth / ((1 + k_left**2)**0.5)
            y_center_single = (pose_x-x_center_single) * \
                k_left+y_center_single
            x_center_single = pose_x

        k = 0
        theata = math.atan(k)
        x_center_single2 = math.cos(
            theata)*x_center_single+math.sin(theata)*y_center_single
        y_center_single2 = - \
            math.sin(theata)*x_center_single+math.cos(theata)*y_center_single
        print("laser point(%f, %f)" %
              (x_center_single2, y_center_single2))
        if abs(y_center_single2)>0.3:#0.3
              y_center_single2 = 0.3*y_center_single2/abs(y_center_single2)
        if debug:
            plt.show()

        
        D_cam_rear = 0.32
        wheel_base = 0.570  # wheel base
        k_tuning = 50  # tuing factor
        steerAngle = math.atan(2 * wheel_base * y_center_single2 / (y_center_single2 *
                                                                    y_center_single2 + (x_center_single2 + D_cam_rear) * (x_center_single2 + D_cam_rear)))

        cam_cmd.angular.z = k_tuning * steerAngle
	#if (len(pointcloud_left_x) > point_num) & (len(pointcloud_right_x) > point_num):
            #flag_laser = 1
	#else:
	    #flag_laser = 2
        flag_laser = 1
        #flag_laserPub.publish(flag_laser)
        cam_cmd.angular.x = flag_laser
        print("laser   steerAngle=", steerAngle)
        cmdPub.publish(cam_cmd)

    elif cross_flag:
        left_angle = 45
        right_angle = 45
        x = []
        y = []
        for i in range(1440):
            if (i > 4*left_angle) & (i < 1440 - 4*right_angle):
                continue
            if math.isinf(msg.ranges[i]) or (msg.ranges[i] < 1.5):
                continue
            angle = math.pi /  720 * i
            x_tmp = msg.ranges[i] * math.cos(angle)
            y_tmp = msg.ranges[i] * math.sin(angle)
            x.append(x_tmp)
            y.append(y_tmp)
        if len(x) != 0:
            index_left = y.index(max(y))
            index_right = y.index(min(y))
            print("Obstacle is detected, the left point is (%f, %f), the right point is (%f, %f)" % (
                x[index_left], y[index_left], x[index_right], y[index_right]))


if __name__ == "__main__":
    try:
        rospy.init_node('laser_detection', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, laserCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
