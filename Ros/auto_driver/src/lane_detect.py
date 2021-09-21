#!/usr/bin/python
# -*- coding: utf-8 -*-
from lane_utils import laneDetect
import numpy as np
import cv2
from std_msgs.msg import Float32
import rospy

# 初始化车道线检测常量
frameWidth = 1280  # 宽
frameHeight = 720  # 高
# 透视变换
src_points = np.array([[6, 716], [8, 342], [1263, 363], [1275, 717]], dtype="float32")
dst_points = np.array([[524, 714], [11, 345], [1271, 348], [762, 710]], dtype="float32")
Mwarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算
#相机内参
camMat = np.array([[6.678151103217834e+02, 0, 6.430528691213178e+02],
                    [0, 7.148758960098705e+02, 3.581815819255082e+02], [0, 0, 1]])  # 相机校正矩阵
camDistortion = np.array([[-0.056882894892153, 0.002184364631645, -0.002836821379133, 0, 0]])  # 相机失真矩阵
# 视觉处理
kerSz = (5, 5)  # 膨胀与腐蚀核大小
grayThr = 160  # 二值化阈值
roiXRatio = 0.4  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置
# 划窗检测
winWidth = 200  # 窗的宽度
winNum = 20  # 窗的数目
winThr = 8  # 单条车道线需有8个框检出车道线
pixThr = 200  # 最小连续像素，小于该长度的被舍弃以去除噪声影响
# 距离映射
roadWidCm = 80  # 道路宽度 单位：cm
roadWidPix = 450  # 透视变换后车道线像素数
isShow = False  # 是否返回可视化图片


def main():
    rospy.init_node('lane_node', anonymous=True)
    cap = cv2.VideoCapture('/dev/video10')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)  # 设置读入图像宽
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)  # 设置读入图像长
    cap.set(cv2.CAP_PROP_FPS, 20)    # 设置读入帧率
    print("[Lane Node]: Init")
    # 车道线检测对象
    laneDet = laneDetect(Mwarp, camMat, camDistortion, kerSz, grayThr, frameHeight, frameWidth, roiXRatio,
                winWidth, winNum, winThr, pixThr, roadWidCm, roadWidPix, isShow)  # 初始化车道线检测

    #--- publish topic
    lane_pub  = rospy.Publisher("/lane_detect/bias", Float32, queue_size=10)

    while not rospy.is_shutdown():
        ret, img = cap.read()
        if ret:
            bias = laneDet.spin(img)
            lane_pub.publish(bias)

if __name__ == '__main__':
    main()

