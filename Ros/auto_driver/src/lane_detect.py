#!/usr/bin/python
# -*- coding: utf-8 -*-
from lane_utils import laneDetect
import numpy as np
import cv2
from bluetooth_bridge.msg import LaneMsg, Sensors
import rospy
from cap_init import CapInit
import threading

# 初始化车道线检测常量
frameWidth = 1280  # 宽
frameHeight = 720  # 高
# 透视变换
src_points = np.array([[236, 545], [510, 399], [812, 387], [1162, 544]], dtype="float32")
dst_points = np.array([[414, 706], [446, 441], [868, 430], [863, 708]], dtype="float32")
Mwarp = cv2.getPerspectiveTransform(src_points, dst_points)  # 透视变换矩阵计算
#相机内参
camMat = np.array([[6.678151103217834e+02, 0, 6.430528691213178e+02],
                    [0, 7.148758960098705e+02, 3.581815819255082e+02], [0, 0, 1]])  # 相机校正矩阵
camDistortion = np.array([[-0.056882894892153, 0.002184364631645, -0.002836821379133, 0, 0]])  # 相机失真矩阵
# 视觉处理
kerSz = (5, 5)  # 膨胀与腐蚀核大小
grayThr = 160  # 二值化阈值
roiXRatio = 0.7  # 统计x方向上histogram时选取的y轴坐标范围，以下方底边为起始点，比例定义终止位置
# 划窗检测
winWidth = 200  # 窗的宽度
winNum = 20  # 窗的数目
winThr = 8  # 单条车道线需有8个框检出车道线
pixThr = 200  # 最小连续像素，小于该长度的被舍弃以去除噪声影响
# 距离映射
roadWidCm = 80  # 道路宽度 单位：cm
roadWidPix = 850  # 透视变换后车道线像素数
isShow = False  # 是否返回可视化图片
vip = 3

slope_flag = 0 # 0表示平地，1表示下坡

def main():
    rospy.init_node('lane_node', anonymous=True)
    cap = CapInit()

    print("[Lane Node]: Init")
    # 车道线检测对象
    laneDet = laneDetect(Mwarp, camMat, camDistortion, kerSz, grayThr, frameHeight, frameWidth, roiXRatio,
                winWidth, winNum, winThr, pixThr, roadWidCm, roadWidPix, isShow, vip)  # 初始化车道线检测
    #--- subscribe topic

    def sensor_callback(msg):
        """ 上下坡后重置 """
        global slope_flag
        theta = msg.thetax
        if theta < -1700 and slope_flag==0: # 检测到开始下坡
            slope_flag = 1
        elif abs(theta) < 200 and slope_flag==1 # 检测到下坡结束
            slope_flag = 0
            laneDet.refresh()

    rospy.Subscriber('/vcu', Sensors, sensor_callback)

    #--- publish topic
    lane_pub  = rospy.Publisher("/lane_detect", LaneMsg, queue_size=10)

    ros_spin = threading.Thread(target = rospy.spin)
    ros_spin.setDaemon(True)
    ros_spin.start()

    while not rospy.is_shutdown():
        ret, img = cap.read()
        if ret:
            bias, slope = laneDet.spin(img)
            lane_pub.publish(LaneMsg(bias=bias, slope=slope))

if __name__ == '__main__':
    main()

