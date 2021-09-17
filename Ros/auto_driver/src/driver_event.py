# !/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from PID import PID

""" 事件基类 """
class DriverEvent(object):
    def __init__(self, driver):
        self.driver = driver # 小车驱动

    def is_start(self):
        raise NotImplemented

    def is_end(self):
        raise NotImplemented

    def strategy(self):
        raise NotImplemented

""" 循线事件 """
class FollowLaneEvent(DriverEvent):
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