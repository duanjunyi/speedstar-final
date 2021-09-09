#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  9 16:49:57 2020

@author: hyh
"""
import cv2
import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

cap = cv2.VideoCapture('/dev/video10')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cameraMatrix = np.array([[767.933491065341, 0, 661.829272006519], [0, 767.792106129209, 350.803808338661], [0, 0, 1]])
cameraDistortion = np.array([[0.0147, 0.0593, 0, 0, 0]])
    
T2 = np.array([[1, 0, 0, -96.1797], [0, 0, -1, 118.2930], [0, 1, 0, 409.7199], [0, 0, 0, 1]])
roadWidth = 450

img_width = 1280
img_height = 720

grayThresh = 190#190                547
src_points = np.array([[185,479], [647,386], [869,380], [1277,470]], dtype="float32")
dst_points = np.array([[360., 710], [360., 35.], [920., 35], [920., 710.]], dtype="float32")
M = cv2.getPerspectiveTransform(src_points, dst_points)
m_inv = cv2.getPerspectiveTransform(dst_points, src_points)
point_color = (0, 255, 0) # BGR
thickness = 1
pre = 0.0 
pre_pre = 0.0
time1 = 0.0

lane_direction = Twist()
laser_flag = 0
start_time = 0.0
start_flag = -1
start_state = -1

def lasercallback(msg):
    global laser_flag    
    laser_flag = msg.angular.x
    print("22222222222222222",laser_flag)
    if laser_flag == 1:
        pre = 0.0
        pre_pre = 0.0
rospy.Subscriber("laser_direction", Twist, lasercallback)

def  startcallback(msg):
    global start_flag,start_time
    if msg.data==0 and start_state==-1:
	start_time = time.time()
	start_state = 0 
rospy.Subscriber("/hilens", Int32, startcallback)


def pixel2camera(T, mtx, u, v):
    Matrix = np.zeros([3, 4])
    for i in range(3):
        for j in range(3):
            Matrix[i][j] = mtx[i][j]
    M = np.matmul(Matrix, T)
    xw = (M[0][1] * M[1][3] - M[0][3] * M[1][1] + M[1][1] * M[2][3] * u - M[1][3] * M[2][1] * u - M[0][1] * M[2][3] * v + M[0][3] * M[2][1] * v) /\
         (M[0][0] * M[1][1] - M[0][1] * M[1][0] + M[1][0] * M[2][1] * u - M[1][1] * M[2][0] * u - M[0][0] * M[2][1] * v + M[0][1] * M[2][0] * v)
    yw = -(M[0][0] * M[1][3] - M[0][3] * M[1][0] + M[1][0] * M[2][3] * u - M[1][3] * M[2][0] * u - M[0][0] * M[2][3] * v + M[0][3] * M[2][0] * v) /\
         (M[0][0] * M[1][1] - M[0][1] * M[1][0] + M[1][0] * M[2][1] * u - M[1][1] * M[2][0] * u - M[0][0] * M[2][1] * v + M[0][1] * M[2][0] * v)
    worldPosition = np.array([[xw], [yw], [0], [1]])
    cameraPosition = np.matmul(T, worldPosition)
    return cameraPosition[0][0], cameraPosition[1][0], cameraPosition[2][0]

def camera2pixel(cameraMatrix, x, y, z):
    u = cameraMatrix[0][0] * x / z + cameraMatrix[0][2]
    v = cameraMatrix[1][1] * y / z + cameraMatrix[1][2]
    return u, v

#cameraMatrix = np.array([[508.2868, 0, 338.0616], [0, 510.0489, 240.6656], [0, 0, 1]])
#cameraDistortion = np.array([[0.0059, -0.0117, 0, 0, 0]])
#rotationMatrix = np.array([[-0.9991, 0.0258, 0.0327], [0.0313, -0.0545, 0.9980], [0.0276, 0.9982, 0.0536]])
#translationVector = np.array([[63.5395], [111.1884], [356.0019]])
#T = np.array([[-0.9991, 0.0258, 0.0327, 63.5395], [0.0313, -0.0545, 0.9980, 111.1884], [0.0276, 0.9982, 0.0536, 356.0019], [0, 0, 0, 1]])

def cam():
    global cap, M, m_inv, point_color, thickness,laser_flag
    global pre,pre_pre,start_time,time1
    global cameraMatrix, cameraDistortion,roadWidth,T2,img_width,img_height,grayThresh
    cmdPub = rospy.Publisher('lane_direction', Twist, queue_size=1)

    cam_cmd = Twist()
    
    buffer = 0
    #img = cv2.imread("img.png", cv2.IMREAD_COLOR)
    #dir_path = "D:\\Pictures\\"
#    print("0st")
    if cap.isOpened():
        #file_name = dir_path + str(i+1) + ".jpg"
	print("camera open:       11111111111111111111111111111111111111111111111111111 ")
        if laser_flag == 2 :
            return
        start = time.time()
        ret, img = cap.read()

        #img = cv2.undistort(img, cameraMatrix, cameraDistortion, None, cameraMatrix)
        gray_Blur = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_Blur[0:int(img_height * 0.5 + 70), :] = 0
        # cv2.imshow('gray', gray_Blur)
        kernel = np.ones((3, 3), np.uint8)
        # gray_Blur = cv2.dilate(gray_Blur, kernel, iterations = 1)
        gray_Blur = cv2.erode(gray_Blur, kernel, iterations=1)
        gray_Blur = cv2.dilate(gray_Blur, kernel, iterations=1)
        
        origin_thr = np.zeros_like(gray_Blur)
        origin_thr[(gray_Blur >= grayThresh)] = 255
        
        binary_warped = cv2.warpPerspective(origin_thr, M, (1280, 720), cv2.INTER_LINEAR)
        histogram = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)  # 4/5
        histogramLow = np.sum(binary_warped[int(binary_warped.shape[0] * 0.9):, :], axis=0)
        lane_base = np.argmax(histogram)
	mean = np.mean(histogram)
	lane_base2 = lane_base
        midpoint = int(histogram.shape[0] / 2)
        
        leftx_base = np.argmax(histogramLow[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        
        step = 80
        letfList = []
        leftNum = 0
        righList = []
        rightNum = 0
        leftxLowList = []
        rightxLowList = []
        rightxList = []
        
        sumStep = 0
        sumLeft = 0
        
        for i in range(histogramLow.shape[0]):
            sumLeft = sumLeft + histogramLow[i]
            if i % step == 0:
                if i < midpoint:
                    if sumLeft > 20000:
                        leftxLowList.append(int(i - step * 0.4))
                else:
                    if sumLeft > 20000:
                        rightxLowList.append(int(i - step * 0.4))
                sumLeft = 0
        
        for i in range(histogram.shape[0]):
            sumStep = sumStep + histogram[i]
            if i % step == 0:
                if i < midpoint:
                    if sumStep > 20000:
                        leftNum = leftNum + 1
                else:
                    if sumStep > 20000:
                        rightNum = rightNum + 1
                        rightxList.append(int(i - step * 0.4))
                sumStep = 0
        
        # find the left and right
        if len(rightxList) != 0:
            if len(rightxLowList) != 0:
                if abs(rightxList[-1] - rightxLowList[-1]) < 100:
                    lane_base = int(rightxList[-1] * 0.3 + rightxLowList[-1] * 0.7)
                    # direct_flag = 1
                else:
                    lane_base = rightxLowList[-1]
            else:
                lane_base = rightxList[-1]
        else:
            if len(rightxLowList) != 0:
                lane_base = rightxLowList[-1]
            else:
                if len(leftxLowList) != 0:
                    lane_base = leftxLowList[-1]
                else:
                    if buffer > 3:
                        buffer = 0
                    else:
                        buffer = buffer + 1
        
        if len(leftxLowList) != 0:
            leftx_base = leftxLowList[-1]
        nwindows = 25
        window_height = int(binary_warped.shape[0] / nwindows)
        # find which pixel is notzero
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
	print ("histogram mean: ", mean, ";lane_base: ", lane_base,"lane_base2:",lane_base2)
	#if(mean>4000):
	    #lane_base=lane_base2
        lane_current = lane_base
        margin = 70
        minpix = 25
        
        lane_inds = []
        for window in range(nwindows):
        
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            if window == 0:
                win_x_low = lane_current - margin
                win_x_high = lane_current + margin
            else:
                win_x_low = lane_current - margin
                win_x_high = lane_current + margin
            # the region satisfy the number take_first
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                    nonzerox < win_x_high)).nonzero()[0]
            lane_inds.append(good_inds)
        
            if len(good_inds) > minpix:
                lane_current = int(np.mean(nonzerox[good_inds]))
            else:
                if window > 6:
                    break
        
        lane_inds = np.concatenate(lane_inds)
        
        pixelX = nonzerox[lane_inds]
        pixelY = nonzeroy[lane_inds]
        if len(pixelX) < 15:
	    cam_cmd.angular.x = 0
            cam_cmd.angular.z = pre
	    cmdPub.publish(cam_cmd)
            return False
        
        a2, a1, a0 = np.polyfit(pixelY, pixelX, 2)
        
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        fitx = a2 * ploty ** 2 + a1 * ploty + a0
        pt_z = np.ones(ploty.shape[0])
        pt_m = np.vstack((fitx, ploty, pt_z))
        pt_minv = np.dot(m_inv, pt_m)
        x_inv = pt_minv[0, :] / pt_minv[2, :]
        y_inv = pt_minv[1, :] / pt_minv[2, :]
        
        k = (y_inv[img_height - 1] - y_inv[0]) / (x_inv[img_height - 1] - x_inv[0])
        #print ("k: ", k)
	#print (np.min(x_inv), np.max(x_inv))
	if (np.max(x_inv) > 2000):
	    cam_cmd.angular.x = 0
            cam_cmd.angular.z = pre
	    cmdPub.publish(cam_cmd)
	    return False
        if (np.min(x_inv) > 1000):
            flag = 1
        elif (np.max(x_inv) < 400):
            flag = -1
        elif (k > 0) & (k < 1):
            flag = 1
        elif (k < 0) & (k > -1):
            flag = -1
        else:
	    cam_cmd.angular.x = 0
            cam_cmd.angular.z = pre
	    cmdPub.publish(cam_cmd)
            return False
        if flag == 1:
            print ("right")
        else:
            print ("left")
        
        a2, a1, a0 = np.polyfit(y_inv, x_inv, 2)
        y_average = np.average(y_inv)
        x_intertcept = 2 * a2 * y_average + a1
        x_average = a2 * y_average ** 2 + a1 * y_average +a0
        
        xc = np.zeros((int(np.shape(x_inv)[0] / 10), 1))
        yc = np.zeros((int(np.shape(x_inv)[0] / 10), 1))
        zc = np.zeros((int(np.shape(x_inv)[0] / 10), 1))
        # print("cost time1: ", time.time() - start)
        for i in range(int(x_inv.shape[0] / 10)):
            #cv2.circle(img, (int(x_inv[i * 10]), int(y_inv[i * 10])), 5, point_color, thickness)
            # if i < 5:
            #     xc, yc, zc = position_3D(T, cameraMatrix, x_inv[i], y_inv[i])
            xc2, yc2, zc2 = pixel2camera(T2, cameraMatrix, x_inv[i * 10], y_inv[i * 10])
        #    xc3, yc3, zc3 = position_3D(T, cameraMatrix, y_inv[i * 10], x_inv[i * 10])
        #    print (xc2-xc3, yc2-yc3, zc2-zc3)
            #u, v = camera2pixel(cameraMatrix, xc2, yc2, zc2)
        #    cv2.circle(img, (int(u), int(v)), 5, point_color, thickness)
        #    print (x_inv[i * 10], y_inv[i * 10], u, v)
            xc[i] = xc2
            yc[i] = yc2
            zc[i] = zc2
        step = int(xc.shape[0] / 31)
        x_center = []
        z_center = []
        for i in range(10):
            index = (i + 20) * step + 10
            z_intertcept = (zc[index+1] - zc[index-1])/(xc[index+1] - xc[index-1])
            if z_intertcept > 0:
                x_center_tmp = xc[index] - flag * roadWidth / ((z_intertcept**2 + 1)**0.5)*z_intertcept
                z_center_tmp = zc[index] + flag * roadWidth / ((z_intertcept ** 2 + 1) ** 0.5)
            else:
                x_center_tmp = xc[index] + flag * roadWidth / ((z_intertcept**2 + 1)**0.5)*z_intertcept
                z_center_tmp = zc[index] - flag * roadWidth / ((z_intertcept ** 2 + 1) ** 0.5)
            y_center = yc[index]
            x_center.append(x_center_tmp)
            z_center.append(z_center_tmp)
    #        print ("K: ", z_intertcept)
            #print ("3D: ", index, " ", x_center_tmp, z_center_tmp)
            #u1, v1 = camera2pixel(cameraMatrix, x_center, y_center, z_center)
            #u2, v2 = camera2pixel(cameraMatrix, xc[index], y_center, zc[index])
    #        print ("2D: ", u2, v2, u1, v1)
            #cv2.circle(img, (int(u1), int(v1)), 5, (0, 0, 255), thickness)
            #cv2.circle(img, (int(u2), int(v2)), 5, point_color, thickness)
        
        D_cam_rear = 625
        wheel_base = 570  # wheel base
        #k_tuning = -21 # tuing factor
        k_tuning = -28
        index_test = 5
        if flag == 1:
	    k_tuning = -21 #21
            index_test = 5 #5
	    #print("right,right,right",k_tuning)


        steerAngle = math.atan(2 * wheel_base * x_center[index_test] / ( x_center[index_test]*x_center[index_test] + (z_center[index_test] + D_cam_rear) * (z_center[index_test] + D_cam_rear)))
        print("***********lane*******",x_center[index_test],"   ",z_center[index_test])
        cam_cmd.angular.x = 1
        cam_cmd.angular.z = k_tuning * steerAngle
        tmp_angle = cam_cmd.angular.z
        #print("steerAngle=", cam_cmd.angular.z)
        if abs(x_center[index_test])>1200:
	    cam_cmd.angular.x = 0
            cam_cmd.angular.z = pre
	    cmdPub.publish(cam_cmd)
            return
        
        if pre!=0.0 and pre_pre != 0.0 and time.time() - start_time > 6:
            if (abs(tmp_angle - pre) > 5.0) and (tmp_angle/abs(tmp_angle) != pre/abs(pre)) and (tmp_angle/abs(tmp_angle) != pre_pre/abs(pre_pre)):
	        cam_cmd.angular.x = 0
                cam_cmd.angular.z = pre
	        cmdPub.publish(cam_cmd)
                return 
	    #if (abs(x_center[index_test]/z_center[index_test]) > 0.6) and (tmp_angle/abs(tmp_angle) != pre/abs(pre)) and (tmp_angle/abs(tmp_angle) != pre_pre/abs(pre_pre)):
	        #cam_cmd.angular.x = 0
                #cam_cmd.angular.z = pre
	        #cmdPub.publish(cam_cmd)
                #return 
            #if (abs(tmp_angle - pre) > 7):
             #   return
	
	
        pre_pre = pre
        pre = cam_cmd.angular.z 
        cmdPub.publish(cam_cmd)
        #print ("cost time: ", time.time()-start)
        
                # print (xc, yc, zc, xc2, yc2 ,zc2)
#        print ("cost time: ", time.time()-start)
        cv2.imshow('img2', img)
        #cv2.waitKey(20)
#        k = cv2.waitKey(20)
    # q键退出
        return True
#        if k & 0xff == ord('q'):
#            print("3st return")
#            return 

if __name__ == '__main__':
    rospy.init_node('lane_vel', anonymous=True)
    rate = rospy.Rate(10)  
    #global time1
    time1 = time.time()
    try:
        print(rospy.is_shutdown())  # FALSE
        while not rospy.is_shutdown():
            #global cap
	    #print(' 1cam.spin ==')
            cam()
            #print(' cam.spin ==')
            rate.sleep()
        cap.release()
    	#cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass

        


