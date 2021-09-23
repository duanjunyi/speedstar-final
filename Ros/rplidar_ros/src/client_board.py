#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import numpy as np
# from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from bluetooth_bridge.msg import BoardMsg
import threading
import cv2

imSize = (520, 520)
flag = 0
bias = 0
angle = 0

""" sub topics """
topic_scan = "/scan"
def callback_scan(scan):
    img = np.zeros(imSize, dtype=np.uint8)
    ranges = np.array(scan.ranges)  # [1440,]
    ranges[ranges>2.5] = 0
    angle = np.arange(len(scan.ranges)) * scan.angle_increment
    px = ( np.cos(angle) * ranges * 100 ).astype(np.int32) + 260
    py = ( np.sin(angle) * ranges * 100 ).astype(np.int32) + 260
    img[px, py] = 255
    img_show = process(img)
    cv2.imshow('scan', img_show)
    cv2.waitKey(5)

def process(img):
    global flag
    global bias
    global angle

    img_show = np.dstack((img, img, img))
    # 剪裁
    img_roi = img[260-80:260+20, 260-80:260+80]
    img_show_roi = img_show[260-80:260+20, 260-80:260+80]
    cv2.rectangle(img_show, (170, 170), (350, 290), (0, 255, 0), 2)

    # 霍夫变换检测直线
    lines = cv2.HoughLines(img_roi, 4.5, np.pi/180, 70, 40)
    if lines is None:
        return img_show
    lines = lines[:,0,:]
    if len(lines)<2: # 判断：线数少于2
        print('线数<2')
        flag = 0
        return img_show

    # 分成左右两侧
    lines_dist = np.abs(lines[:,0])
    sort_idx = np.argsort(lines_dist) # 按照 dist 排序
    lines = lines[sort_idx]
    lines_dist = lines_dist[sort_idx]
    d_dist = lines_dist[1:] - lines_dist[:-1] # (len-1,)
    mid_idx = np.argmax(d_dist) + 1 # 找最大rho增量对应的idx作为分割点
    if d_dist[mid_idx-1] < 30: # 判断：两堆之间差异不大
        print('分不出两份')
        flag = 0
        return img_show

    line1 = meanline(lines[:mid_idx])
    line2 = meanline(lines[mid_idx:])



    drawline(img_show_roi, line1, (0,0,255))
    drawline(img_show_roi, line2, (255,0,0))
    print(lines)
    #for line in lines:
    #    drawline(img_show_roi, line, (0,0,255))
    #for line in lines[mid_idx:]:
     #   drawline(img_show_roi, line1, (255,0,0))

    # 计算原点到过中心点的平行线的距离 rho
    # center_pt = (80, 80)
    theta = (line1[1] + line2[1]) / 2
    if theta == 0:
        rho = 80
    elif theta < np.pi/2:  # y = k(x-80) + 80,  rho > 0
        k = -np.cos(theta) / np.sin(theta)
        rho = distance((0,0), a=k, b=-1,c=80-80*k)
    elif theta == np.pi/2:
        print('有障碍物')
        return img_show
    elif theta < np.pi: # rho < 0
        k = -np.cos(theta) / np.sin(theta)
        rho = -distance((0,0), a=k, b=-1,c=80-80*k)

    # 取 line1 和 line2 锐夹角的角平分线
    if np.abs(line1[0]) < np.abs(rho) and np.abs(line2[0]) > np.abs(rho):
        line_mid = meanline(np.vstack([line1, line2]))
        pos_bias = np.abs(rho) - np.abs(line_mid[0])
        # 将line_mid头朝上
        if line_mid[1]>np.pi/2:
            line_mid[1] -= np.pi
            line_mid[0] = -line_mid[0]
        ang_bias = -line_mid[1]
        print('偏差 pos_bias=%5.2f, ang_bias=%5.2f' % (pos_bias, ang_bias))
    return img_show

def distance(pt, a, b, c):
    """ 点 pt 到直线 ax+by+c=0 的距离"""
    return np.abs( a*pt[0] + b*pt[1] + c) / np.sqrt(a**2 + b**2)

def drawline(img, line, color, linewidth=2):
    rho, theta = line
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))
    cv2.line(img, (x1,y1), (x2,y2), color, linewidth)


def meanline(lines):
    """ 对线做平均 """
    # 判断线是否是接近竖直，如果不是，可以直接平均
    if np.all(np.logical_or(lines[:, 1] > np.pi*4/5, lines[:, 1] < np.pi/5)):
        # 接近竖直的线，全转化为头朝上
        idx = (lines[:, 1] > np.pi/2).nonzero()[0]
        lines[idx, 1] -= np.pi
        lines[idx, 0] = -lines[idx, 0]
        # 进行平均
        mline = np.mean(lines, axis=0, keepdims=False)
        # 平均完后，可能会出现 <0 的情况
        if mline[1] < 0:
            mline[1] += np.pi
            mline[0] = -mline[0]
    else:
        mline = np.mean(lines, axis=0, keepdims=False)

    return mline


def main():

    # --- node init
    rospy.init_node('lidar_board', anonymous=True)
    print("[lidar_board]: Init")

    # --- publisher topic
    board_detection_pub = rospy.Publisher('/lida/board_detect', BoardMsg, queue_size=100)
    rate = rospy.Rate(10)
    # --- subscriber topic
    rospy.Subscriber(topic_scan, LaserScan, callback_scan)
    # --- subscriber thread
    thread_spin = threading.Thread(target=rospy.spin)
    thread_spin.start()

    while not rospy.is_shutdown():
        # --- subscriber topic

        rate.sleep()
        board_detection_pub.publish(BoardMsg(flag = flag, bias = bias, angle = angle))
        rospy.loginfo("flag = %d, bias = %4.2f, angle = %4.2f", flag, bias, angle)


if __name__ == '__main__':
    main()