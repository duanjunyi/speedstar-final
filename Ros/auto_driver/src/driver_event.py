# !/usr/bin/python
# -*- coding: utf-8 -*-

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
    def __init__(self, driver, controller):
        super(FollowLaneEvent, self).__init__(driver)
        self.controller = controller  # 例如 pid 控制器

    def is_start(self):
        return True

    def is_end(self):
        return False

    def strategy(self):
        bias = self.driver.get_bias()
        direct = self.controller(bias)
        self.driver.set_direction(direct)