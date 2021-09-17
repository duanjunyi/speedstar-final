#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
import os
BASE_DIR = os.path.abspath(os.path.dirname(__file__))
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
frameFps = 20  # 帧率
camMat = np.array([[6.678151103217834e+02, 0, 6.430528691213178e+02],
                   [0, 7.148758960098705e+02, 3.581815819255082e+02], [0, 0, 1]])  # 相机校正矩阵
camDistortion = np.array([[-0.056882894892153, 0.002184364631645, -0.002836821379133, 0, 0]])  # 相机失真矩阵


# 透视变换
src_points = np.array([[0., 527.], [416., 419.], [781., 420.], [1065., 542.]], dtype="float32")  # 源点
dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")  # 目标点
# src_points = np.array([[498., 596.], [789., 596.], [250., 720.], [1050., 720.]], dtype="float32")  # 源点
# dst_points = np.array([[300., 100.], [980., 100.], [300., 720.], [980., 720.]], dtype="float32")  # 目标点
MWarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算


# 视觉处理
kerSz = (3, 3)  # 膨胀与腐蚀核大小
grayThr = 125  # 二值化阈值
roiXRatio = 0.4  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置
nwindows = 20  # 窗的数目
window_width = 200  # 窗的宽度
minpix = 200  # 最小连续像素，小于该长度的被舍弃以去除噪声影响


# 距离映射
x_cmPerPixel = 90 / 665.00  # x方向上一个像素对应的真实距离 单位：cm
y_cmPerPixel = 81 / 680.00  # y方向上一个像素对应的真实距离 单位：cm
roadWidth = 80  # 道路宽度 单位：cm
y_offset = 50.0  # 由于相机位置较低，识别到的车道线距离车身较远，不是当前位置，定义到的车道线与车身距离 单位：cm<no usage>
cam_offset = 18.0  # 相机中心与车身中轴线的距离 单位：cm


class camera:
    def __init__(self):
        self.camMat = camMat   # 相机校正矩阵
        self.camDistortion = camDistortion  # 相机失真矩阵
        self.cap = cv2.VideoCapture(BASE_DIR + '\\video\\challenge_video.mp4')  # 读入视频
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)  # 设置读入图像宽
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)  # 设置读入图像长
        self.cap.set(cv2.CAP_PROP_FPS, frameFps)    # 设置读入帧率
        self.kernal = np.ones(kerSz, np.uint8)      # 定义膨胀与腐蚀的核
        self.frame_h = frameHeight
        self.frame_w = frameWidth
        self.win_w = window_width                   # 窗宽
        self.win_h = int(self.frame_h // nwindows)  # 窗高
        self.win_n = nwindows                       # 窗数
        self.lane_xc = np.zeros((2, self.win_n)).astype(np.int32)  # 左右车道线中心点x坐标，lane_center_x[0]为左，[1]为右
        self.lane_yc = np.arange(int(self.frame_h - 0.5*self.win_h), 0, -self.win_h)  # 车道线中心点 y 坐标
        self.lane_flag = np.full((2, self.win_n), False, np.bool)  # 左右车道线 检出标志位
        self.lane_curve = [None, None]                             # 左右拟合曲线
        self.wins_thr = 8            # 车道线检出需满足 检出窗数 > wins_thr
        self.pix_thr = minpix        # 一个窗中检出车道线需满足 非零像素 > minpix
        self.show = True
        self.first_frame = True
        self.lane_cmPerPixel = x_cmPerPixel  # 车道线内部一个像素对应的真实距离 单位：cm


    def __del__(self):
        self.cap.release()  # 释放摄像头

    def init_lane_xc(self, img):
        """ 输入第一帧经过预处理的图片，初始化 lane_xc 中的第一列，即左右车道线底部两个窗的 xc 坐标 """
        self.first_frame = False
        histogram_x = np.sum(img[int(img.shape[0] * 0.8):, :], axis=0)  # 计算 x方向直方图 [x,]
        midpoint = int(histogram_x.shape[0] / 2)                                      # x方向中点，用来判断左右
        self.lane_xc[0, 0] = int(np.argmax(histogram_x[:midpoint]))                   # 定义左车道线的基点
        self.lane_xc[1, 0] = int(np.argmax(histogram_x[midpoint:]) + midpoint)        # 定义右车道线的基点
        self.lane_flag[:, 0] = [True, True]

    def spin(self):
        ret, img = self.cap.read()  # 读入图片
        # img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_LINEAR)
        if ret == True:
            #--- 校正，二值化，透视变化
            img_prep = self.prepocess(img)
            if self.show:
                img_show = np.repeat(img_prep[:, :, None], 3, axis=2)

            # 处理第一帧，为迭代做准备
            if self.first_frame:
                self.init_lane_xc(img_prep)

            #--- 迭代更新 lane_xc
            l_det_pts = []              # 保存检出的车道线像素，用于拟合
            r_det_pts = []
            pixes = img_prep.nonzero()  # 所有非零像素
            for i in range(self.win_n):
                # 窗中心
                l_win_xc = self.get_win_xc(i, 0)    # 左车道线第i个窗中心 x 坐标
                r_win_xc = self.get_win_xc(i, 1)
                win_yc = self.lane_yc[i]            # 车道线第i个窗中心 y 坐标
                # 生成窗
                l_win_pts = self.get_win(pixes, xc=l_win_xc, yc=win_yc)  # 左窗中所有像素点坐标 [n,2]
                r_win_pts = self.get_win(pixes, xc=r_win_xc, yc=win_yc)  # 右窗
                # 绘制窗
                if self.show:
                    cv2.rectangle(  img_show,
                                    (int(l_win_xc-self.win_w/2), int(win_yc-self.win_h/2)),
                                    (int(l_win_xc+self.win_w/2), int(win_yc+self.win_h/2)), (127,127,255), 2)  # 在图中画出左车道线的窗
                    cv2.rectangle(  img_show,
                                    (int(r_win_xc-self.win_w/2), int(win_yc-self.win_h/2)),
                                    (int(r_win_xc+self.win_w/2), int(win_yc+self.win_h/2)), (255,127,127), 2)  # 在图中画出右车道线的窗

                # 检测窗中的车道线中心 lane_xc
                l_det = len(l_win_pts) > self.pix_thr   # 检测到左车道线中点
                r_det = len(r_win_pts) > self.pix_thr   # 检测到右车道线中点

                if l_det:
                    l_det_pts.append(l_win_pts)
                    self.lane_xc[0, i] = int(np.mean(l_win_pts[:, 0]))  # 更新左车道线的 x 中心位置
                if r_det:
                    r_det_pts.append(r_win_pts)
                    self.lane_xc[1, i] = int(np.mean(r_win_pts[:, 0]))  # 更新右车道线的 x 中心位置
                self.lane_flag[:, i] = [l_det, r_det]   # 更新检出标志位

            #--- 绘制检测到的车道点
            if self.show:
                xc_idxs = self.lane_flag.nonzero()
                xcs = self.lane_xc[xc_idxs]
                for i in range(len(xcs)):
                    point = (xcs[i], self.lane_yc[xc_idxs[1][i]])
                    cv2.circle(img_show, point, 4, (125,125,255), -1)

            #--- 拟合
            # 取出拟合点 det_pts
            l_win_nums = np.sum(self.lane_flag[0]) # 有效窗口数
            r_win_nums = np.sum(self.lane_flag[1])
            if l_win_nums:
                l_det_pts = np.vstack(l_det_pts)  # 所有检测到的左车道线像素 [n, 2(x, y)]
            if r_win_nums:
                r_det_pts = np.vstack(r_det_pts)  # 所有检测到的右车道线像素 [n, 2(x, y)]
            # 进行拟合
            if l_win_nums>=self.wins_thr and r_win_nums>=self.wins_thr:     # 左右车道线都检测出
                l_curve = np.polyfit(l_det_pts[:, 1], l_det_pts[:, 0], 2)   # 左车道线拟合
                r_curve = np.polyfit(r_det_pts[:, 1], r_det_pts[:, 0], 2)   # 右车道线拟合
                self.update_curve(l_curve, 0, l_det_pts)   # 更新左，右车道拟合线
                self.update_curve(r_curve, 1, r_det_pts)
            elif l_win_nums >= r_win_nums and l_win_nums>0:  # 只检出左车道线
                l_curve = np.polyfit(l_det_pts[:, 1], l_det_pts[:, 0], 2)   # 左车道线拟合
                self.update_curve(l_curve, 0, l_det_pts)
                # self.lane_curve[1] = None
                self.update_lane_xc(1)              # 右车道线已检出的点不可信，用左车道线偏移量将其覆盖

            elif r_win_nums > l_win_nums:   # 只检出右车道线
                r_curve = np.polyfit(r_det_pts[:, 1], r_det_pts[:, 0], 2) # 右车道线拟合
                self.update_curve(r_curve, 1, r_det_pts)
                # self.lane_curve[0] = None
                self.update_lane_xc(0)              # 左车道线已检出的点不可信，用右车道线偏移将其覆盖

            else:
                print('No lane was detected')

            # 计算航向偏差
            if l_win_nums>=self.wins_thr and r_win_nums>=self.wins_thr:
                lx = np.polyval(self.lane_curve[0], self.frame_h)
                rx = np.polyval(self.lane_curve[1], self.frame_h)
                cen_pos = (lx + rx) / 2.0  # 车道中心线位置
                veh_pos = self.frame_w / 2.0  # 小车位置，目前定义为画面中心
                self.lane_cmPerPixel = roadWidth / np.abs(rx - lx)
                distance_from_center = (veh_pos - cen_pos) * self.lane_cmPerPixel
            elif l_win_nums >= r_win_nums and l_win_nums>0:  # 只检出左车道线
                cen_pos = np.polyval(self.lane_curve[0], self.frame_h)*self.lane_cmPerPixel + roadWidth / 2  # 车道中心线位置
                veh_pos = self.frame_w / 2.0  * self.lane_cmPerPixel # 小车位置，目前定义为画面中心
                distance_from_center = veh_pos - cen_pos
            elif r_win_nums > l_win_nums:   # 只检出右车道线
                cen_pos = np.polyval(self.lane_curve[1], self.frame_h)*self.lane_cmPerPixel - roadWidth / 2  # 车道中心线位置
                veh_pos = self.frame_w / 2.0  * self.lane_cmPerPixel # 小车位置，目前定义为画面中心
                distance_from_center = veh_pos - cen_pos

            # 绘制拟合车道线
            if self.show:
                ymax = self.frame_h - 1
                y = np.arange(0, ymax, 2)  # 定义自变量 y
                if  l_win_nums >= self.wins_thr and r_win_nums >= self.wins_thr:
                    l_x = np.polyval(self.lane_curve[0], y)
                    r_x = np.polyval(self.lane_curve[1], y)
                    pts_left = np.transpose(np.vstack([l_x, y])).astype(np.int32)
                    pts_right = np.flipud(np.transpose(np.vstack([r_x, y]))).astype(np.int32)
                    pts = np.vstack((pts_left, pts_right))  # 拟合点
                    # 计算曲率
                    l_curve_real = np.polyfit(y * y_cmPerPixel, l_x * x_cmPerPixel, 2)  # 映射到现实尺度下左车道线的拟合
                    r_curve_real = np.polyfit(y * y_cmPerPixel, r_x * x_cmPerPixel, 2)  # 映射到现实尺度下右车道线的拟合
                    curve_rad = (self.curvature(l_curve_real, ymax) + self.curvature(r_curve_real, ymax)) / 2
                elif l_win_nums >= r_win_nums and l_win_nums>0:
                    l_x = np.polyval(self.lane_curve[0], y)
                    pts = np.transpose(np.vstack([l_x, y])).astype(np.int32)
                    # 计算曲率
                    l_curve_real = np.polyfit(y * y_cmPerPixel, l_x * x_cmPerPixel, 2)  # 映射到现实尺度下左车道线的拟合
                    curve_rad = self.curvature(l_curve_real, ymax)
                elif l_win_nums < r_win_nums:
                    r_x = np.polyval(self.lane_curve[1], y)
                    pts = np.flipud(np.transpose(np.vstack([r_x, y]))).astype(np.int32)
                    # 计算曲率
                    r_curve_real = np.polyfit(y * y_cmPerPixel, r_x * x_cmPerPixel, 2)  # 映射到现实尺度下右车道线的拟合
                    curve_rad = self.curvature(r_curve_real, ymax)
                else:
                    return
                cv2.polylines(img_show, [pts,], False, (0, 255, 0), 2)
                cv2.imshow('img_show', img_show)
                cv2.waitKey(1)

                # 映射到真实图像, 绘图
                color_warp = np.zeros_like(img).astype(np.uint8)
                cv2.fillPoly(color_warp, [pts,], (0, 255, 0))
                newwarp = cv2.warpPerspective(color_warp, MWarp, (img.shape[1], img.shape[0]), None, cv2.WARP_INVERSE_MAP)
                result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
                font = cv2.FONT_HERSHEY_SIMPLEX
                radius_text = "Radius of Curvature: %.3f cm" % (curve_rad)
                cv2.putText(result, radius_text, (100, 100), font, 1, (20, 20, 255), 2)
                pos_flag = 'right' if distance_from_center>0 else 'left'
                center_text = "Vehicle is %.3f cm %s of center" % (abs(distance_from_center), pos_flag)
                cv2.putText(result, center_text, (100, 150), font, 1, (20, 20, 255), 2)
                cv2.imshow('result', result)
                cv2.waitKey(1)
                cv2.waitKey(0)


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
        gray_img_1c = np.mean(gray_img, axis=2).astype(np.uint8)  # 单通道化
        perspect_img = cv2.warpPerspective(gray_img_1c, MWarp, (gray_Blur.shape[1], gray_Blur.shape[0]),
                                            cv2.INTER_LINEAR)  # 透视变换
        return perspect_img


    def get_win(self, pixes, xc, yc):
        """
        从图中取出一个窗中所有像素点的位置, [n, 2(x, y)]
        pixes: 一张图中所有非零像素坐标
        """
        py, px = pixes
        idx = ( (py >= yc-self.win_h//2) & (py < yc + self.win_h//2) & \
                (px >= xc-self.win_w//2) & (px < xc + self.win_w//2) )
        return np.hstack([px[idx][:, None], py[idx][:, None]])

    def get_win_xc(self, i, side):
        """
        获得第i个窗的x坐标，其来源有如下几个（按照优先级排序）：
        1. 若上一帧第 i 个窗检测到车道线，沿用x坐标
        2. 若上一帧未检出，采用本帧第 i-1 个窗的x坐标
        3. 若本帧第 i-1 个窗未检出，采用上一帧拟合曲线计算的x坐标
        4. 若不存在拟合曲线（第一帧不存在上一帧），用已检测出的点拟合
        side: 0-left, 1-right
        """
        if self.lane_flag[side, i]:
            xc = self.lane_xc[side, i]
        elif i>0 and self.lane_flag[side, i-1]:
            xc = self.lane_xc[side, i-1]
        elif self.lane_curve[side] is not None:
            xc = np.polyval(self.lane_curve[side], self.lane_yc[i])
        else: # 先拟合
            det_y = self.lane_yc[self.lane_flag[side]]
            det_x = self.lane_xc[side][self.lane_flag[side]]
            self.lane_curve[side] = np.polyfit(det_y, det_x, 2)
            xc = np.polyval(self.lane_curve[side], self.lane_yc[i])
        return int(xc)

    def update_lane_xc(self, side):
        """ 更新车道线中点 """
        if side == 1:
            self.lane_xc[1, :] = self.lane_xc[0, :] + 850
            self.lane_flag[1, :] = True
        elif side == 0:
            self.lane_xc[0, :] = self.lane_xc[1, :] - 850
            self.lane_flag[0, :] = True

    def update_curve(self, curve_new, side, det_pts):
        """ 更新拟合曲线，防止曲线突变 """
        if self.lane_curve[side] is None:
            self.lane_curve[side] = curve_new
        diff = self.lane_curve[side][1] * curve_new[1]  # y=0时的梯度 dx/dy 不能突变
        centers = np.sum(self.lane_flag[side])          # 检出点数
        if diff < -0 and centers < 10:
            return
        ymid = - curve_new[1] / curve_new[0] / 2
        print(curve_new, ymid)
        cnt = 0
        while ymid > 100 and ymid<self.frame_h-100 and abs(curve_new[0])> 3e-3 :
            num = len(det_pts)
            det_pts = det_pts[int(0.2*num):]
            curve_new = np.polyfit(det_pts[:, 1], det_pts[:, 0], 2)
            if cnt < 2:
                cnt += 1
                print('pass')
                continue
            return

        self.lane_curve[side] = curve_new

    def curvature(self, curve, y):
        """ 计算曲率半径 """
        return ((1 + (2*curve[0] * y * y_cmPerPixel + curve[1])**2)**1.5)\
                        / max(np.absolute(2*curve[0]), 0.001)


if __name__ == '__main__':
    cam = camera()
    while True:
        cam.spin()

