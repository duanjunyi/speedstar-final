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
# src_points = np.array([[0., 527.], [416., 419.], [781., 420.], [1065., 542.]], dtype="float32")  # 源点
# dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")  # 目标点
src_points = np.array([[498., 596.], [789., 596.], [250., 720.], [1050., 720.]], dtype="float32")  # 源点
dst_points = np.array([[300., 100.], [980., 100.], [300., 720.], [980., 720.]], dtype="float32")  # 目标点
MWarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算


# 视觉处理
kerSz = (3, 3)  # 膨胀与腐蚀核大小
grayThr = 125  # 二值化阈值
roiXRatio = 2/5  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置
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
<<<<<<< HEAD
        self.cap = cv2.VideoCapture(str(BASE_DIR / 'video/challenge_video2.mp4'))  # 读入视频
=======
        self.cap = cv2.VideoCapture(str(BASE_DIR / 'challenge_video3.mp4'))  # 读入视频
>>>>>>> bdc5dda74e7c77f2ab768bf835c5016d331d615d
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)  # 设置读入图像宽
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)  # 设置读入图像长
        self.cap.set(cv2.CAP_PROP_FPS, frameFps)  # 设置读入帧率
        self.kernal = np.ones(kerSz, np.uint8)  # 定义膨胀与腐蚀的核
        self.win_w = window_width
        self.win_h = int(frameHeight // nwindows)
        self.win_n = nwindows
        self.l_lane_centers = np.zeros((self.win_n, 2)).astype(np.int32)  # 左车道线中心点
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

        pixes = img.nonzero()
        for i in range(self.win_n):
            win_yc = int(h - (i + 0.5) * self.win_h)
            l_win_pts = self.get_win(pixes, xc=l_win_xc, yc=win_yc)  # 左窗
            r_win_pts = self.get_win(pixes, xc=r_win_xc, yc=win_yc)  # 右窗

            if len(l_win_pts):
                l_win_xc = int(np.mean(l_win_pts[:, 0]))   # 更新左车道线下一个窗的x中心位置

            if len(r_win_pts):
                r_win_xc = int(np.mean(r_win_pts[:, 0]))  # 更新右车道线下一个窗的x中心位置

            # 记录检测到的中心点
            self.l_lane_centers[i, :] = [l_win_xc, win_yc] # 记录左车道线窗的中点 cx, cy
            self.r_lane_centers[i, :] = [r_win_xc, win_yc] # 记录右车道线窗的中点 cx, cy


    def spin(self):
        ret, img = self.cap.read()  # 读入图片
        img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_LINEAR)
        if ret == True:
            #--- 校正，二值化，透视变化
            binary_warped = self.prepocess(img)
            h, w = binary_warped.shape[:2]
            if self.first_frame:  # 处理第一帧
                self.init_lane_centers(binary_warped)
                return

            if self.show:
                binary_show = binary_warped.copy()
            # 用更多的点进行拟合：
            l_fit_pts = []
            r_fit_pts = []

            #--- 更新 lane_xc
            pixes = binary_warped.nonzero()
            for i in range(self.win_n):
                # 窗中心等于上一帧的 lane_center
                l_win_xc = self.l_lane_centers[i, 0]    # 上一帧第i个车道中心的x坐标
                r_win_xc = self.r_lane_centers[i, 0]
                win_yc = self.l_lane_centers[i, 1]      # 上一帧第i个车道中心的y坐标
                # 生成窗
                l_win_pts = self.get_win(pixes, xc=l_win_xc, yc=win_yc)  # 左窗中所有像素点坐标 [n,2]
                r_win_pts = self.get_win(pixes, xc=r_win_xc, yc=win_yc)  # 右窗
                if self.show: # 绘制窗
                    cv2.rectangle(  binary_show,
                                    (int(l_win_xc-self.win_w/2), int(win_yc-self.win_h/2)),
                                    (int(l_win_xc+self.win_w/2), int(win_yc+self.win_h/2)), 255, 2)  # 在图中画出左车道线的窗
                    cv2.rectangle(  binary_show,
                                    (int(r_win_xc-self.win_w/2), int(win_yc-self.win_h/2)),
                                    (int(r_win_xc+self.win_w/2), int(win_yc+self.win_h/2)), 255, 2)  # 在图中画出右车道线的窗
                # 计算窗中的 lane_xc
                # 若检测到车道线，用平均值更新中点，否则，不更新 TODO：拟合出下一个点

                l_det = len(l_win_pts) > self.minpix  # 检测到车道线
                r_det = len(r_win_pts) > self.minpix
                self.l_flag[i] = l_det
                self.r_flag[i] = r_det
                if l_det:
                    l_fit_pts.append(l_win_pts)
                    l_lane_xc = int(np.mean(l_win_pts[:, 0]))  # 计算左车道线窗的x中心位置
                if r_det:
                    r_fit_pts.append(r_win_pts)
                    r_lane_xc = int(np.mean(r_win_pts[:, 0]))  # 计算右车道线窗的x中心位置
                if l_det and not r_det:
                    r_lane_xc = r_win_xc + (l_lane_xc - l_win_xc)
                if r_det and not l_det:
                    l_lane_xc = l_win_xc + (r_lane_xc - r_win_xc)
                if not l_det and not r_det:
                    continue

                self.l_lane_centers[i, 0] = l_lane_xc  # 更新
                self.r_lane_centers[i, 0] = r_lane_xc

            #--- 绘制检测到的车道点
            l_lane_det = self.l_lane_centers[self.l_flag]
            r_lane_det = self.r_lane_centers[self.r_flag]

            if self.show:
                # for i in range(self.win_n):
                for point in l_lane_det:
                    cv2.circle(binary_show, point, 4, 125, -1)
                for point in r_lane_det:
                    cv2.circle(binary_show, point, 4, 125, -1)
                cv2.imshow('binary_show', binary_show)
                cv2.waitKey(1)

            #--- 拟合
            l_valid_wins = len(l_lane_det) # 有效窗口数
            r_valid_wins = len(r_lane_det)
            if l_valid_wins:
                l_fit_pts = np.vstack(l_fit_pts)  # 所有检测到的左车道线像素 [n, 2(x, y)]
            if r_valid_wins:
                r_fit_pts = np.vstack(r_fit_pts)  # 所有检测到的右车道线像素 [n, 2(x, y)]
            ymax = binary_warped.shape[0]-1
            y = np.arange(0, ymax, 2)  # 定义自变量 y
            wins_thr = 4
            if l_valid_wins>=wins_thr and r_valid_wins>=wins_thr:
                l_curve = np.polyfit(l_fit_pts[:, 1], l_fit_pts[:, 0], 2)  # 左车道拟合
                r_curve = np.polyfit(r_fit_pts[:, 1], r_fit_pts[:, 0], 2)  # 右车道拟合
                l_x = np.polyval(l_curve, y)  # 计算拟合后左车道线的x坐标
                r_x = np.polyval(r_curve, y)  # 计算拟合后右车道线的x坐标
            elif l_valid_wins > r_valid_wins:
                l_curve = np.polyfit(l_fit_pts[:, 1], l_fit_pts[:, 0], 2)  # 左车道拟合
                l_x = np.polyval(l_curve, y)  # 计算拟合后左车道线的x坐标
                r_curve = self.offset_curve(l_curve, y, l_x, 1)
                r_x = np.polyval(r_curve, y)
            elif l_valid_wins < r_valid_wins:
                r_curve = np.polyfit(r_fit_pts[:, 1], r_fit_pts[:, 0], 2)  # 左车道拟合
                r_x = np.polyval(r_curve, y)  # 计算拟合后左车道线的x坐标
                l_curve = self.offset_curve(r_curve, y, r_x, 1)
                l_x = np.polyval(l_curve, y)
            else:
                raise "fault"

            left_fit_real = np.polyfit(y * y_cmPerPixel, l_x * x_cmPerPixel, 2)  # 映射到现实尺度下左车道线的拟合
            right_fit_real = np.polyfit(y * y_cmPerPixel, r_x * x_cmPerPixel, 2)  # 映射到现实尺度下右车道线的拟合

            l_curve_rad = ((1 + (2*left_fit_real[0]*ymax*y_cmPerPixel + left_fit_real[1])**2)**1.5)\
                            / max(np.absolute(2*left_fit_real[0]), 0.001)   # 左车道线曲率半径
            r_curve_rad = ((1 + (2*right_fit_real[0]*ymax*y_cmPerPixel + right_fit_real[1])**2)**1.5)\
                            / max(np.absolute(2*right_fit_real[0]), 0.001)  # 右车道线曲率半径
            curve_rad = (l_curve_rad + r_curve_rad) / 2  # 整体曲率半径
            lane_width = np.absolute(l_x[-1] - r_x[-1])  # 车道线的像素宽度
            lane_cmPerPixel = roadWidth / lane_width  # 车道线的像素比例
            cen_pos = ((l_x[-1] + r_x[-1]) * lane_cmPerPixel) / 2.0  # 车道中心线位置
            veh_pos = binary_warped.shape[1] * lane_cmPerPixel / 2.0  # 小车位置，目前定义为画面中心，但是摄像头与小车中轴线不一定重合，需要校准
            distance_from_center = veh_pos - cen_pos  # 离中心距离，<0位于左边, >0位于右边

            # 绘图显示
            color_warp = np.zeros_like(img).astype(np.uint8)
            pts_left = np.transpose(np.vstack([l_x, y])).astype(np.int32)
            pts_right = np.flipud(np.transpose(np.vstack([r_x, y]))).astype(np.int32)
            pts = np.vstack((pts_left, pts_right))
            cv2.fillPoly(color_warp, [pts,], (0, 255, 0))
            cv2.imshow('result1', color_warp)
            cv2.waitKey(1)
            newwarp = cv2.warpPerspective(color_warp, MWarp, (img.shape[1], img.shape[0]), None, cv2.WARP_INVERSE_MAP)
            result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
            font = cv2.FONT_HERSHEY_SIMPLEX
            radius_text = "Radius of Curvature: %scm" % (round(curve_rad))
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
        gray_Blur = cv2.dilate(undist_img, self.kernal, iterations = 1)  # 膨胀
        # gray_Blur = cv2.erode(undist_img, self.kernal, iterations=1)  # 腐蚀
        _, gray_img = cv2.threshold(gray_Blur, grayThr, 255, cv2.THRESH_BINARY) # 二值化
        gray_img = np.mean(gray_img, axis=2).astype(np.uint8)  # 单通道化
        perspect_img = cv2.warpPerspective(gray_img, MWarp, (gray_Blur.shape[1], gray_Blur.shape[0]),
                                            cv2.INTER_LINEAR)  # 透视变换
        return perspect_img


    def get_win(self, pixes, xc, yc):
        """
        从图中取出一个窗中所有像素点的位置, [n, 2(x, y)]
        """
        py, px = pixes
        idx = ( (py >= yc-self.win_h//2) & (py < yc + self.win_h//2) & \
                (px >= xc-self.win_w//2) & (px < xc + self.win_w//2) )
        return np.hstack([px[idx][:, None], py[idx][:, None]])

    def offset_curve(self, curve1, y1, x1, direction):
        """
        给定一个曲线，返回其等距曲线方程
        curve1: 给定曲线方程
        y1, x1: 曲线上的点
        direction: curve2在curve1左侧-1，或者右侧1
        """
        dist = 200
        grad = np.concatenate([-2*curve1[0]*y1[None,:]-curve1[1], np.ones((1, len(y1)))], axis=0)
        grad = direction * grad / np.linalg.norm(grad, axis=0) # 单位化梯度
        y2 = y1 + dist * grad[0, :]
        x2 = x1 + dist * grad[1, :]
        # 拟合
        curve2 = np.polyfit(y2, x2, 2)
        return curve2



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


