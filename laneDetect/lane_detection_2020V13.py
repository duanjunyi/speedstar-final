#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import cv2

import numpy as np
import math
import matplotlib.pyplot as plt



#距离映射
x_cmPerPixel = 90/665.00
y_cmPerPixel = 81/680.00
roadWidth = 665

y_offset = 50.0 #cm

#轴间距
I = 58.0
#摄像头坐标系与车中心间距
D = 18.0
#计算cmdSteer的系数
k = -19

class camera:
    def __init__(self):

        self.camMat = []
        self.camDistortion = []

        #self.cap = cv2.VideoCapture('/dev/video10')
        self.cap = cv2.VideoCapture('challenge_video.mp4')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)



 #       self.cam_cmd.linear.x = -0.85



        self.aP = [0.0, 0.0]
        self.lastP = [0.0, 0.0]
        self.Timer = 0
        self.angularScale=6
        self.abc=0
        self.angularz=0

    def __del__(self):
        self.cap.release()

    def calibrateAndTrans(self):
        cameraMatrix = np.array(
            [[5.027859993e+02, 0, 3.5289228025e+02], [0, 5.0422397659e+02, 2.1590340464e+02], [0, 0, 1]])
        cameraDistortion = np.array([[0.04821081, -0.10794579, 0.00025604, 0.00160635, 0.09035667]])
        if cameraMatrix != []:
            self.camMat = cameraMatrix
            self.camDistortion = cameraDistortion
            print
            'CALIBRATION SUCCEEDED!'
        else:
            print
            'CALIBRATION FAILED!'
        return 0

    def spin(self):
        ret, img = self.cap.read()
        if ret == True:
            #img = cv2.imread("challenge_video_Moment.jpg")

            #undstrt = cv2.undistort(img, self.camMat, self.camDistortion, None, self.camMat)
            #img=cv2.flip(img, 1)

            kernel = np.ones((3,3), np.uint8)
            #gray_Blur = cv2.dilate(gray_Blur, kernel, iterations = 1)
            gray_Blur = cv2.erode(img, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_Blur)
            # origin_thr[(gray_Blur <= 100)] = 255
            origin_thr[(gray_Blur >= 125)] = 255
            src_points = np.array([[0., 527.], [416., 419.], [781., 420.], [1065., 542.]], dtype="float32")
            #src_points = np.array([[3,570], [387,460], [906,452], [1041,485]], dtype="float32")
            dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            binary_warped = cv2.warpPerspective(origin_thr, M, (1280, 720), cv2.INTER_LINEAR)
            #binary_warped = cv2.warpPerspective(origin_thr, M, img.shape[1::-1], flags=cv2.INTER_LINEAR)
            #cv2.imshow('binary_warped',binary_warped)
            histogram = np.sum(binary_warped[int(binary_warped.shape[0] /2):, :], axis=0)  # 4/5
    #        histogram1 = np.sum(binary_warped[(binary_warped.shape[0] //5)*4:, :], axis=0)  # 4/5
    #        midpoint = int(histogram1.shape[0]/2)
           # histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
            histogram_y = np.sum(binary_warped[0:binary_warped.shape[0], :], axis=1)# 4/5
            midpoint_y = 320 #int(histogram.shape[0]/2)
            upper_half_histSum = np.sum(histogram_y[0:midpoint_y])
            lower_half_histSum = np.sum(histogram_y[midpoint_y: ])
            try:
                hist_sum_y_ratio = (upper_half_histSum)/(lower_half_histSum)
            except:
                print('hehe')
                hist_sum_y_ratio = 1
            print("=====")
            print(hist_sum_y_ratio)

            lane_base = np.argmax(histogram)
            #lane_base_f = np.argmax(histogram_y)
            midpoint = int(histogram.shape[0]/2)
            '''
            if lane_base >= midpoint:
                LorR = -1.25
                #-1:R
                # 1:L
            else:
                #...
                #LorR = -1

                if hist_sum_y_ratio < 0.1:
                    LorR = -1.25
                    lane_base = 1280 - lane_base
                else:
                    LorR = 1
             '''

            lane_base = np.argmax(histogram)
            nwindows = 10
            window_height = int(binary_warped.shape[0] / nwindows)
            nonzero = binary_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            lane_current = lane_base
            margin = 100
            minpix = 25
            # minpix = 50

            lane_inds = []

            for window in range(nwindows):
                win_y_low = binary_warped.shape[0] - (window + 1) * window_height
                win_y_high = binary_warped.shape[0] - window * window_height
                win_x_low = lane_current - margin
                win_x_high = lane_current + margin
                cv2.rectangle(binary_warped, (win_x_low, win_y_low), (win_x_high, win_y_high),
                              (255, 255, 255), 2)
                cv2.rectangle(binary_warped, (win_x_low, win_y_low), (win_x_high, win_y_high),
                              (255, 255, 255), 2)
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                            nonzerox < win_x_high)).nonzero()[0]

                lane_inds.append(good_inds)
                if len(good_inds) > minpix:
                    lane_current = int(np.mean(nonzerox[good_inds]))  ####
                elif window>=3:
                    break
            cv2.imshow('pic', binary_warped)
            # cv2.imshow('pic',img)
            #cv2.waitKey(1)
            lane_inds = np.concatenate(lane_inds)

            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]

            # calculate the aimPoint
            if (pixelX.size == 0):
                return

            a2, a1, a0 = np.polyfit(pixelY,pixelX, 2)
            A=np.polyfit(pixelY,pixelX, 2)
            x1=np.linspace(200,300,50)
            y1=np.polyval(A,x1)
            #plt.plot(y1,x1)
            #plt.show()
            try:
                b1,b0=np.polyfit(pixelY,pixelX,1)

                point_min = (int(b1*np.min(pixelX)+b0), int(np.min(pixelX)))
                point_max = (int(b1*np.max(pixelX)+b0), int(np.max(pixelX)))
                scope = (point_min[1] - point_max[1]) / (point_min[0] - point_max[0])
                #cv2.putText(binary_warped, "left: " + str(scope), (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

                cv2.line(binary_warped, (point_min), (point_max), color=(0, 0, 0), thickness=15)
                cv2.line(binary_warped, (point_min), (point_max), color=(255, 255, 255), thickness=10)
                cv2.line(binary_warped, (point_min), (point_max), color=(0, 0, 0), thickness=5)

                dis=abs((b0-640+b1*720)/((b1*b1+1)**0.5))

                y0=720
                x0=a2*y0*y0+a1*y0+a0
                y1=720
                x1=640
                dis1=abs(x0-x1)
                cv2.putText(binary_warped, "* dist_to_lane:" + str(dis), (640, 718), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                cv2.putText(binary_warped, "# dist_to_point:" + str(dis1), (640, 688), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

                cv2.putText(binary_warped, "dis: " + str(dis), (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

            except:
                pass
            aveX = np.average(pixelX)
            # 区分左右车道线,以计算截距
            '''
            if (2 * a2 * aveX + a1) > 0:  # 斜率大于0
                if a2 > 0:
                        # x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                    x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)  # 求截距
                
                else:
                    x_intertcept= (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)


            else:  # 斜率小于0
                if a2 > 0:
                        # x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                    x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)
                
                else:
                    x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)


            if (x_intertcept > 599):
                # if (x_intertcept > 1279):
                LorR = -1.25# RightLane
                # print('R')
           # else:
            #    LorR = 1.0 # LeftLane
                # print('L')
            '''

            #LorR = -1.25
            frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]
            aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

            # 计算aimLaneP处斜率，从而得到目标点的像素坐标
            lanePk = 2 * a2 * aimLaneP[0] + a1
            #if (lanePk > 500 or lanePk < -500):
            if(abs(lanePk)<0.1):
                if lane_base >= midpoint:
                    LorR = -1.25
                    #-1:R
                    # 1:L
                else:
                    #...
                    #LorR = -1

                    if hist_sum_y_ratio < 0.1:
                        LorR = -1.25
                        #lane_base = 1280 - lane_base
                    else:
                        LorR = 0.8
                self.aP[0] = aimLaneP[0] +LorR * roadWidth / 2
                self.aP[1] = aimLaneP[1]
            else:
                if (2 * a2 * aveX + a1) > 0:  # 斜率大于0
                    if a2 > 0:
                            # x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)  # 求截距

                    else:
                        x_intertcept= (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)


                else:  # 斜率小于0
                    if a2 > 0:
                            # x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                    else:
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)


                if (x_intertcept > 599):
                    # if (x_intertcept > 1279):
                    LorR = -1.4# RightLane
                    # print('R')
                else:
                    LorR = 0.8 # LeftLane

                k_ver = -1 / lanePk

                theta = math.atan(k_ver)
                self.aP[0] = aimLaneP[0] + math.cos(theta) * (LorR) * roadWidth / 2
                self.aP[1] = aimLaneP[1] + math.sin(theta) * (LorR) * roadWidth / 2

            self.aP[0] = (self.aP[0] - 599) * x_cmPerPixel
            self.aP[1] = (680 - self.aP[1]) * y_cmPerPixel + y_offset

            # 计算目标点的真实坐标
            if (self.lastP[0] > 0.001 and self.lastP[1] > 0.001):
                if (((self.aP[0] - self.lastP[0]) ** 2 + (
                        self.aP[1] - self.lastP[1]) ** 2 > 2500) and self.Timer < 2):  # To avoid the mislead by walkers
                    self.aP = self.lastP[:]
                    self.Timer += 1
                else:
                    self.Timer = 0

            self.lastP = self.aP[:]
            steerAngle = math.atan(2 * I * self.aP[0] / (self.aP[0] * self.aP[0] + (self.aP[1] + D) * (self.aP[1] + D)))


            self.angularz=k * steerAngle
    #        print("K=", k )
            #print("steerAngle=", steerAngle)
            cv2.imshow('real_world', img)
            cv2.imshow('pic', binary_warped)
            # cv2.imshow('pic',img)
            cv2.waitKey(10)
            # else:


if __name__ == '__main__':



    try:
        cam = camera()
        cam.calibrateAndTrans()
        while True:
            cam.spin()


    except :
        print("helloworld")
        pass


