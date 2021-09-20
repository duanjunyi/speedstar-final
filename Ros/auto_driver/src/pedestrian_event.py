# !/usr/bin/python
# -*- coding: utf-8 -*-

from PID import PID
from driver_event import DriverEvent

class PedestrianEvent(DriverEvent):
    '''
    斑马线策略
    is_start: 目标检测到斑马线，四个条件需要同时满足：
             (1)box面积大于0.4w*0.15h
             (2)斑马线label的score>0.9
             (3)斑马线位于图片的下方，即y_min>0.6h
             (4)连续1个输出满足上述要求
    is_end: is_start条件任意一个不满足则is_end
    strategy: 直接刹车速度为0
    '''
    def __init__(self, driver, scale_prop, score_limit, y_limit):
        super(PedestrianEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit


    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(1)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag & (score >= self.score_limit) & (scale >= 1) & (y_min >= self.y_limit * height):
            return True
        return False

    def is_end(self):
        return not self.is_start()

    def strategy(self):
        self.driver.set_speed(0)

class SpeedLimitedEvent(DriverEvent):
    '''
    限速策略
    is_start: 目标检测到限速标志，四个条件需要同时满足：
            (1)box面积大于
    '''
    def __init__(self, driver, scale_prop, score_limit, y_limit, speed):
        super(SpeedLimitedEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit
        self.set_speed = speed

    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(3)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag & (score >= self.score_limit) & (scale >= 1) & (y_min <= self.y_limit * height):
            return True
        return False

    def is_end(self):
        return not self.is_start()

    def strategy(self):
        self.driver.set_speed(self.set_speed)


class SpeedUnlimitedEvent(DriverEvent):
    '''
    解除策略
    is_start: 目标检测到限速标志，四个条件需要同时满足：
            (1)box面积大于
    '''
    def __init__(self, driver, scale_prop, score_limit, y_limit, speed):
        super(SpeedUnlimitedEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit
        self.set_speed = speed

    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(5)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag & (score >= self.score_limit) & (scale >= 1) & (y_min <= self.y_limit * height):
            return True
        return False

    def is_end(self):
        return not self.is_start()

    def strategy(self):
        self.driver.set_speed(self.set_speed)


class SpeedMinimumEvent(DriverEvent):
    '''
    解除策略
    is_start: 目标检测到限速标志，四个条件需要同时满足：
            (1)box面积大于
    '''
    def __init__(self, driver, scale_prop, score_limit, y_limit, speed):
        super(SpeedMinimumEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit
        self.set_speed = speed

    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(4)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag & (score >= self.score_limit) & (scale >= 1) & (y_min <= self.y_limit * height):
            return True
        return False

    def is_end(self):
        return not self.is_start()

    def strategy(self):
        self.driver.set_speed(self.set_speed)



