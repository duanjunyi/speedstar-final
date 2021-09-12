#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import cv2

import numpy as np
import math
import matplotlib.pyplot as plt


# 常量定义
# 摄像头
frameWidth = 1280  # 宽
frameHeight = 720  # 长
frameFps = 30  # 帧率
camMat = []  # 相机校正矩阵
camDistortion = []  # 相机失真矩阵


# 透视变换
src_points = np.array([[0., 527.], [416., 419.], [781., 420.], [1065., 542.]], dtype="float32")  # 源点
dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")  # 目标点
MWarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算


# 视觉处理
kerSz = (3, 3)  # 膨胀与腐蚀核大小
grayThr = 125  # 二值化阈值
roiXRatio = 3/5  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置
nwindows = 10  # 窗的数目
window_width = 200  # 窗的宽度
minpix = 25  # 最小连续像素，小于该长度的被舍弃以去除噪声影响


# 距离映射
x_cmPerPixel = 90 / 665.00  # x方向上一个像素对应的真实距离 单位：cm
y_cmPerPixel = 81 / 680.00  # y方向上一个像素对应的真是距离 单位：cm
roadWidth = 80  # 道路宽度 单位：cm
y_offset = 50.0  # 由于相机位置较低，识别到的车道线距离车身较远，不是当前位置，定义到的车道线与车身距离 单位：cm<no usage>
cam_offset = 18.0  # 相机中心与车身中轴线的距离 单位：cm


# 控制
I = 58.0  # 轴间距<no usage>
k = -19  # 计算cmdSteer的系数<no usage>


class camera:
    def __init__(self):
        self.camMat = camMat   # 相机校正矩阵
        self.camDistortion = camDistortion  # 相机失真矩阵
        self.cap = cv2.VideoCapture('challenge_video.mp4')  # 读入视频
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)  # 设置读入图像宽
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)  # 设置读入图像长
        self.cap.set(cv2.CAP_PROP_FPS, frameFps)  # 设置读入帧率

        #       self.cam_cmd.linear.x = -0.85


    def __del__(self):
        self.cap.release()  # 释放摄像头


    def spin(self):
        ret, img = self.cap.read()  # 读入图片
        if ret == True:


            # 预处理，图像增强
            # undistimg = cv2.undistort(img, self.camMat, self.camDistortion, None, self.camMat)  # 校正畸变图像
            mask = np.zeros_like(img)  # 创建遮罩
            cv2.rectangle(mask, (0, int(img.shape[0] * (1 - roiXRatio))), (img.shape[1], img.shape[0]), (255, 255, 255), cv2.FILLED)  # 填充遮罩
            segment = cv2.bitwise_and(img, mask)  # 取出遮罩范围
            kernel = np.ones(kerSz, np.uint8)  # 定义膨胀与腐蚀的核
            # gray_Blur = cv2.dilate(gray_Blur, kernel, iterations = 1)  # 膨胀
            gray_Blur = cv2.erode(segment, kernel, iterations=1)  # 腐蚀
            origin_thr = np.zeros_like(gray_Blur)
            origin_thr[(gray_Blur >= grayThr)] = 255  # 二值化
            binary_warped = cv2.warpPerspective(origin_thr, MWarp, (gray_Blur.shape[1], gray_Blur.shape[0]),
                                                cv2.INTER_LINEAR)  # 透视变换
            histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] * roiXRatio):, :], axis=0)  # 计算x方向直方图


            # 滑窗识别车道线
            midpoint = int(histogram_x.shape[0] / 2)  # x方向中点，用来判断左右
            #  nwindows = 10  # 窗的数目
            window_height = int(binary_warped.shape[0] * (1 - roiXRatio) / nwindows)  # 窗的高度
            #  window_width = 200  # 窗的宽度
            nonzero = binary_warped.nonzero()  # 非零像素索引
            nonzeroy = np.array(nonzero[0])  # 非零像素y坐标
            nonzerox = np.array(nonzero[1])  # 非零像素x坐标
            left_base = np.argmax(histogram_x[:midpoint]) / 3  # 定义左车道线的基点
            right_base = int(np.argmax(histogram_x[midpoint:]) / 3) + midpoint  # 定义右车道线的基点
            left_current = left_base  # 左车道线当前窗x中心位置
            right_current = right_base  # 右车道线当前窗x中心位置
            #  minpix = 25  # 最小连续像素，小于该长度的被舍弃以去除噪声影响
            left_inds = []  # 所有被识别为左车道线的像素索引
            right_inds = []  # 所有被识别为右车道线的像素索引

            for window in range(nwindows):
                win_y_low = binary_warped.shape[0] - (window + 1) * window_height  # 窗的下方坐标
                win_y_high = binary_warped.shape[0] - window * window_height  # 窗的上方坐标
                win_x_left_low = int(left_current - window_width / 2)  # 左车道线窗的左方坐标
                win_x_left_high = int(left_current + window_width / 2)  # 左车道线窗的右方坐标
                win_x_right_low = int(right_current - window_width / 2)  # 右车道线窗的左方坐标
                win_x_right_high = int(right_current + window_width / 2) # 右车道线窗的右方坐标
                cv2.rectangle(binary_warped, (win_x_left_low, win_y_low), (win_x_left_high, win_y_high),
                              (255, 255, 255), 2)  # 在图中画出左车道线的窗
                cv2.rectangle(binary_warped, (win_x_right_low, win_y_low), (win_x_right_high, win_y_high),
                              (255, 255, 255), 2)  # 在图中画出右车道线的窗
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_left_low) & (
                        nonzerox < win_x_left_high)).nonzero()[0]  # 处在左车道线窗中的非零像素索引
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_right_low) & (
                        nonzerox < win_x_right_high)).nonzero()[0]  # 处在右车道线窗中的非零像素索引
                left_inds.append(good_left_inds)
                right_inds.append(good_right_inds)
                if len(good_left_inds) > minpix:
                    left_current = int(np.mean(nonzerox[good_left_inds]))  # 更新左车道线窗的x中心位置
                if len(good_right_inds) > minpix:
                    right_current = int(np.mean(nonzerox[good_right_inds]))  # 更新右车道线窗的x中心位置
            cv2.imshow('binary_warped', binary_warped)  # 显示每一帧窗的位置
            cv2.waitKey(1)
            left_inds = np.concatenate(left_inds)
            right_inds = np.concatenate(right_inds)
            leftx = nonzerox[left_inds]  # 左车道线x坐标
            lefty = nonzeroy[left_inds]  # 左车道线y坐标
            rightx = nonzerox[right_inds]  # 右车道线x坐标
            righty = nonzeroy[right_inds]  # 右车道线y坐标


            # 拟合
            left_fit = np.polyfit(lefty, leftx, 2)  # 左车道拟合
            right_fit = np.polyfit(righty, rightx, 2)  # 右车道拟合
            y = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])  # 定义自变量
            ymax = int(np.max(y))
            leftx_fit = np.polyval(left_fit, y)  # 计算拟合后左车道线的x坐标
            rightx_fit = np.polyval(right_fit, y)  # 计算拟合后右车道线的x坐标
            left_fit_real = np.polyfit(y * y_cmPerPixel, leftx_fit * x_cmPerPixel, 2)  # 映射到现实尺度下左车道线的拟合
            right_fit_real = np.polyfit(y * y_cmPerPixel, rightx_fit * x_cmPerPixel, 2)  # 映射到现实尺度下右车道线的拟合
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
            color_warp = np.zeros_like(binary_warped).astype(np.uint8)
            pts_left = np.array([np.transpose(np.vstack([leftx_fit, y]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([rightx_fit, y])))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
            newwarp = cv2.warpPerspective(color_warp, MWarp, (img.shape[1], img.shape[0]), None, cv2.WARP_INVERSE_MAP)
            result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
            font = cv2.FONT_HERSHEY_SIMPLEX
            radius_text = "Radius of Curvature: %scm" % (round(curverad))
            if distance_from_center > 0:
                pos_flag = 'right'
            else:
                pos_flag = 'left'
            cv2.putText(result, radius_text, (100, 100), font, 1, (255, 255, 255), 2)
            center_text = "Vehicle is %.3fcm %s of center" % (abs(distance_from_center), pos_flag)
            cv2.putText(result, center_text, (100, 150), font, 1, (255, 255, 255), 2)
            cv2.imshow('result', result)
            cv2.waitKey(1)





if __name__ == '__main__':

    try:
        cam = camera()
        while True:
            cam.spin()
    except:
        print("helloworld")
        pass


