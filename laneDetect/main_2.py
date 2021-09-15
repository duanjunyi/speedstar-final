#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parent
import time
# ms_time  = lambda: (int(round(time.time() * 1000)))

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt


# 常量定义
# 摄像头
frameWidth = 1280  # 宽
frameHeight = 720  # 长
frameFps = 30  # 帧率
camMat = np.array([[6.678151103217834e+02, 0, 6.430528691213178e+02],
                   [0, 7.148758960098705e+02, 3.581815819255082e+02], [0, 0, 1]])  # 相机校正矩阵
camDistortion = np.array([[-0.056882894892153, 0.002184364631645, -0.002836821379133, 0, 0]])  # 相机失真矩阵


# 透视变换
src_points = np.array([[0., 527.], [416., 419.], [781., 420.], [1065., 542.]], dtype="float32")  # 源点
dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")  # 目标点
MWarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算


# 视觉处理
kerSz = (3, 3)  # 膨胀与腐蚀核大小
grayThr = 145  # 二值化阈值
roiXRatio = 3/5  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置
roiXBase = 0.3  # 统计左右初始窗的y轴范围
nwindows = 15  # 窗的数目
window_width = 200  # 窗的宽度
minpix = 200  # 最小连续像素，小于该长度的被舍弃以去除噪声影响


# 距离映射
x_cmPerPixel = 90 / 665.00  # x方向上一个像素对应的真实距离 单位：cm
y_cmPerPixel = 81 / 680.00  # y方向上一个像素对应的真实距离 单位：cm
roadWidth = 80  # 道路宽度 单位：cm
y_offset = 50.0  # 由于相机位置较低，识别到的车道线距离车身较远，不是当前位置，定义到的车道线与车身距离 单位：cm<no usage>
cam_offset = 18.0  # 相机中心与车身中轴线的距离 单位：cm


# 控制
I = 58.0  # 轴间距<no usage>
k = -19  # 计算cmdSteer的系数<no usage>

""" 单帧处理过程
图像预处理
生成基点：
    第一帧，计算左右基点
    从第二帧开始，从上一帧继承基点
currentx = 基点
迭代，求出每个窗中车道线中心点：
    生成窗
    更新 currentx:
        if 窗中白点>minpix, 为检测到车道：
            currentx = 统计窗中白色部分 x 平均值 xc_lane
        elif 若右/左侧检测到车道线：
            左/右车道线根据右/左车道线更新
        elif 两侧都没检测到：
            不更新，沿用上一窗中心值
        (TODO 更好的更新方法也许是用前面获得的点拟合出下一点)
        记录xc_lane用于拟合
    可视化

"""

class camera:
    def __init__(self):
        self.camMat = camMat   # 相机校正矩阵
        self.camDistortion = camDistortion  # 相机失真矩阵
        self.cap = cv2.VideoCapture(str(BASE_DIR / 'challenge_video.mp4'))  # 读入视频
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)  # 设置读入图像宽
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)  # 设置读入图像长
        self.cap.set(cv2.CAP_PROP_FPS, frameFps)  # 设置读入帧率
        self.kernal = np.ones(kerSz, np.uint8)  # 定义膨胀与腐蚀的核
        self.win_w = window_width
        self.win_h = int(frameHeight * roiXRatio // nwindows)
        self.win_n = nwindows
        self.l_lane_centers = np.zeros((self.win_n, 2)).astype(np.int32)  # 左车道线中心点，用于拟合
        self.r_lane_centers = np.zeros((self.win_n, 2)).astype(np.int32)  # 右车道线中心点
        self.l_flag = np.full((self.win_n, ), False, np.bool)
        self.r_flag = np.full((self.win_n, ), False, np.bool)
        self.minpix = minpix
        self.show = True
        self.first_frame = True

    def __del__(self):
        self.cap.release()  # 释放摄像头

    def init_lane_centers(self, img):
        """ 输入第一帧经过预处理的图片，初始化 lane_centers """
        self.first_frame = False
        h, w = img.shape
        histogram_x = np.sum(img[int(img.shape[0] * (1-roiXBase)):, :], axis=0)  # 计算 x方向直方图 [x,]
        midpoint = int(histogram_x.shape[0] / 2)                        # x方向中点，用来判断左右
        l_win_xc = int(np.argmax(histogram_x[:midpoint]))                  # 定义左车道线的基点
        r_win_xc = int(np.argmax(histogram_x[midpoint:]) + midpoint)       # 定义右车道线的基点

        for i in range(self.win_n):
            win_yc = int(h - (i + 0.5) * self.win_h)
            l_win = self.get_win(img, xc=l_win_xc, yc=win_yc)  # 左窗
            r_win = self.get_win(img, xc=r_win_xc, yc=win_yc)  # 右窗

            good_l_x = l_win.nonzero()[1]
            good_r_x = r_win.nonzero()[1]
            if len(good_l_x):
                l_win_xc = int(l_win_xc + np.mean(good_l_x) - self.win_w/2)   # 更新左车道线下一个窗的x中心位置

            if len(good_r_x):
                r_win_xc = int(r_win_xc + np.mean(good_r_x) - self.win_w/2)  # 更新右车道线下一个窗的x中心位置

            # 记录检测到的中心点
            self.l_lane_centers[i, :] = [l_win_xc, win_yc] # 记录左车道线窗的中点 cx, cy
            self.r_lane_centers[i, :] = [r_win_xc, win_yc] # 记录右车道线窗的中点 cx, cy


    def spin(self):
        ret, img = self.cap.read()  # 读入图片

        if ret == True:
            #--- 校正，二值化，透视变化
            binary_warped = self.prepocess(img)
            h, w = binary_warped.shape[:2]
            if self.first_frame:  # 处理第一帧
                self.init_lane_centers(binary_warped)
                return

            if self.show:
                binary_show = binary_warped.copy()

            #--- 更新 lane_xc
            for i in range(self.win_n):
                # 窗中心等于上一帧的 lane_center
                l_win_xc = self.l_lane_centers[i, 0]    # 上一帧第i个车道中心的x坐标
                r_win_xc = self.r_lane_centers[i, 0]
                win_yc = self.l_lane_centers[i, 1]      # 上一帧第i个车道中心的y坐标
                # 生成窗
                l_win = self.get_win(binary_warped, xc=l_win_xc, yc=win_yc)  # 左窗
                r_win = self.get_win(binary_warped, xc=r_win_xc, yc=win_yc)  # 右窗
                if self.show: # 绘制窗
                    cv2.rectangle(  binary_show,
                                    (int(l_win_xc-self.win_w/2), int(win_yc-self.win_h/2)),
                                    (int(l_win_xc+self.win_w/2), int(win_yc+self.win_h/2)), 255, 2)  # 在图中画出左车道线的窗
                    cv2.rectangle(  binary_show,
                                    (int(r_win_xc-self.win_w/2), int(win_yc-self.win_h/2)),
                                    (int(r_win_xc+self.win_w/2), int(win_yc+self.win_h/2)), 255, 2)  # 在图中画出右车道线的窗
                # 计算窗中的 lane_xc
                # 若检测到车道线，用平均值更新中点，否则，不更新 TODO：拟合出下一个点
                good_l_x = l_win.nonzero()[1]  # 非零像素 x 坐标
                good_r_x = r_win.nonzero()[1]
                l_det = len(good_l_x) > self.minpix  # 检测到车道线
                r_det = len(good_r_x) > self.minpix
                if l_det:
                    l_lane_xc = int(l_win_xc + np.mean(good_l_x) - self.win_w/2)  # 计算左车道线窗的x中心位置
                if r_det:
                    r_lane_xc = int(r_win_xc + np.mean(good_r_x) - self.win_w/2)  # 计算右车道线窗的x中心位置
                if l_det and not r_det:
                    r_lane_xc = r_win_xc + (l_lane_xc - l_win_xc)
                if r_det and not l_det:
                    l_lane_xc = l_win_xc + (r_lane_xc - r_win_xc)
                if not l_det and not r_det:
                    continue

                self.l_lane_centers[i, 0] = l_lane_xc  # 更新
                self.r_lane_centers[i, 0] = r_lane_xc

            #--- 绘制检测到的车道点
            if self.show:
                for i in range(self.win_n):
                    cv2.circle(binary_show, self.l_lane_centers[i], 4, 125, -1)
                    cv2.circle(binary_show, self.r_lane_centers[i], 4, 125, -1)
                cv2.imshow('binary_show', binary_show)
                cv2.waitKey(1)

            #--- 拟合
            left_fit = np.polyfit(self.l_lane_centers[:,1], self.l_lane_centers[:,0], 2)  # 左车道拟合
            right_fit = np.polyfit(self.r_lane_centers[:,1], self.r_lane_centers[:,0], 2)  # 右车道拟合
            ymax = binary_warped.shape[0]-1
            y = np.linspace(0, ymax, ymax+1)  # 定义自变量 y
            leftx_fit = np.polyval(left_fit, y)  # 计算拟合后左车道线的x坐标
            rightx_fit = np.polyval(right_fit, y)  # 计算拟合后右车道线的x坐标
            left_fit_real = np.polyfit(y * y_cmPerPixel, leftx_fit * x_cmPerPixel, 2)  # 映射到现实尺度下左车道线的拟合
            right_fit_real = np.polyfit(y * y_cmPerPixel, rightx_fit * x_cmPerPixel, 2)  # 映射到现实尺度下右车道线的拟合
            if np.absolute(2*left_fit_real[0])==0 or np.absolute(2*right_fit_real[0])==0:  # 壁免除零
                left_curverad = 1000
                right_curverad = 1000
            else:
                left_curverad = ((1 + (2*left_fit_real[0]*ymax*y_cmPerPixel + left_fit_real[1])**2)**1.5)\
                                / np.absolute(2*left_fit_real[0])  # 左车道线曲率半径
                right_curverad = ((1 + (2*right_fit_real[0]*ymax*y_cmPerPixel + right_fit_real[1])**2)**1.5)\
                             / np.absolute(2*right_fit_real[0])  # 右车道线曲率半径
            curverad = (left_curverad + right_curverad) / 2  # 整体曲率半径
            lane_width = np.absolute(leftx_fit[ymax] - rightx_fit[ymax])  # 车道线的像素宽度
            lane_cmPerPixel = roadWidth / lane_width  # 车道线的像素比例
            cen_pos = ((leftx_fit[ymax] + rightx_fit[ymax]) * lane_cmPerPixel) / 2.0  # 车道中心线位置
            veh_pos = binary_warped.shape[1] * lane_cmPerPixel / 2.0  # 小车位置，目前定义为画面中心，但是摄像头与小车中轴线不一定重合，需要校准
            distance_from_center = veh_pos - cen_pos  # 离中心距离，<0位于左边, >0位于右边

            # 绘图显示
            color_warp = np.zeros_like(img).astype(np.uint8)
            pts_left = np.transpose(np.vstack([leftx_fit, y])).astype(np.int32)
            pts_right = np.flipud(np.transpose(np.vstack([rightx_fit, y]))).astype(np.int32)
            pts = np.vstack((pts_left, pts_right))
            cv2.fillPoly(color_warp, [pts,], (0, 255, 0))
            cv2.imshow('result1', color_warp)
            cv2.waitKey(1)
            newwarp = cv2.warpPerspective(color_warp, MWarp, (img.shape[1], img.shape[0]), None, cv2.WARP_INVERSE_MAP)
            result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
            font = cv2.FONT_HERSHEY_SIMPLEX
            radius_text = "Radius of Curvature: %scm" % (round(curverad))
            cv2.putText(result, radius_text, (100, 100), font, 1, (20, 20, 255), 2)
            pos_flag = 'right' if distance_from_center>0 else 'left'
            center_text = "Vehicle is %.3fcm %s of center" % (abs(distance_from_center), pos_flag)
            cv2.putText(result, center_text, (100, 150), font, 1, (20, 20, 255), 2)
            cv2.imshow('result', result)
            cv2.waitKey(1)

    def prepocess(self, img):
        """
        取下方区域，矫正畸变，二值化，透视变换
        """
        mask = np.zeros_like(img)  # 创建遮罩
        cv2.rectangle(mask, (0, int(img.shape[0] * (1 - roiXRatio))), (img.shape[1], img.shape[0]), (255, 255, 255), cv2.FILLED)  # 填充遮罩
        segment = cv2.bitwise_and(img, mask)  # 取出遮罩范围
        undist_img = cv2.undistort(segment, self.camMat, self.camDistortion, None, self.camMat)  # 校正畸变图像
        # gray_Blur = cv2.dilate(gray_Blur, self.kernel, iterations = 1)  # 膨胀
        gray_Blur = cv2.erode(undist_img, self.kernal, iterations=1)  # 腐蚀
        _, gray_img = cv2.threshold(gray_Blur, grayThr, 255, cv2.THRESH_BINARY) # 二值化
        gray_img = np.mean(gray_img, axis=2).astype(np.uint8)  # 单通道化
        perspect_img = cv2.warpPerspective(gray_img, MWarp, (gray_Blur.shape[1], gray_Blur.shape[0]),
                                            cv2.INTER_LINEAR)  # 透视变换
        return perspect_img


    def get_win(self, img, xc, yc):
        """
        从图中取出一个窗, xc, yc 为窗中心点
        """
        ymax, xmax = img.shape
        half_w = self.win_w // 2
        half_h = self.win_h // 2
        ylow = max(yc-half_h, 0)
        yhigh = min(yc+half_h, ymax)
        xlow = min(max(xc-half_w, 0), xmax)
        xhigh = max(min(xc+half_w, xmax), 0)
        return img[ylow:yhigh, xlow:xhigh]



if __name__ == '__main__':
    cam = camera()
    while True:
        cam.spin()
    try:
        cam = camera()
        while True:
            cam.spin()
    except:
        print("helloworld")
        pass


