# !/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from PID import PID


class DriverEvent(object):
    """ 事件基类 """
    def __init__(self, driver):
        self.driver = driver # 小车驱动

    def is_start(self):
        raise NotImplemented

    def is_end(self):
        raise NotImplemented

    def strategy(self):
        raise NotImplemented


class FollowLaneEvent(DriverEvent):
    """ 循线事件 """
    def __init__(self, driver, controller=None):
        super(FollowLaneEvent, self).__init__(driver)
        # 定义控制器
        if controller is not None:
            self.controller = controller
        else:
            # ------------------- Kp,  Ki,  Kd
            self.controller = PID(5,  0.1, 0.1, setpoint=0, sample_time=0.01)
            self.controller.output_limits = (0, 100)

    def is_start(self):
        """ 事件是否开始 """
        return True

    def is_end(self):
        """ 事件是否终止 """
        return False

    def strategy(self):
        """ 控制策略 """
        bias = self.driver.get_bias()
        direct = self.controller(bias)
        self.driver.set_direction(direct)


class RedStopEvent(DriverEvent):
    """ 红灯事件 """
    def __init__(self, driver, area_thr, y_thr, score_thr=0.5):
        """
        初始化
        :param area_thr: 检测红灯的面积阈值
        :param score_thr: 检测红灯的置信度阈值
        :param y_thr: 红灯高度阈值
        """
        super(FollowLaneEvent, self).__init__(driver)
        self.area_thr = area_thr
        self.score_thr = score_thr
        self.y_thr = y_thr

    def is_start(self):
        """ 事件是否开始 """
        flag, x_min, y_min, x_max, y_max, score = self.driver.get_objs(2)
        if flag and score > self.score_thr:
            area = (x_max - x_min) * (y_max - y_min)
            y = (y_min + y_max) / 2
            if area > self.area_thr and y > self.y_thr:
                return True
        return False

    def is_end(self):
        """ 事件是否终止 """
        return not self.is_start()

    def strategy(self):
        """ 控制策略 """
        self.driver.set_speed(0)


class GreenGoEvent(DriverEvent):
    """ 绿灯事件 """
    def __init__(self, driver, speed, area_thr, y_thr, score_thr=0.5):
        """
        初始化
        :param speed: 启动时设置的速度
        :param area_thr: 检测绿灯的面积阈值
        :param score_thr: 检测绿灯的置信度阈值
        :param y_thr: 绿灯高度阈值
        """
        super(FollowLaneEvent, self).__init__(driver)
        self.speed = speed
        self.area_thr = area_thr
        self.score_thr = score_thr
        self.y_thr = y_thr

    def is_start(self):
        """ 事件是否开始 """
        flag, x_min, y_min, x_max, y_max, score = self.driver.get_objs(0)
        if flag and score > self.score_thr:
            area = (x_max - x_min) * (y_max - y_min)
            y = (y_min + y_max) / 2
            if area > self.area_thr and y > self.y_thr:
                return True
        return False

    def is_end(self):
        """ 事件是否终止 """
        return not self.is_start()

    def strategy(self):
        """ 控制策略 """
        self.driver.set_speed(self.speed)