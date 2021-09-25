#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import time
from PID import PID
import threading
from FuzzyCtr import FollowLineCtr


def loop_idx(size):
    """ 生成循环索引 """
    idx = 0
    while True:
        yield idx
        idx = 0 if idx>=size-1 else idx+1


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
    """
    循线事件
    is_start: 初始化即开启
    is_end: 从不结束
    strategy: 模糊控制方向，延时
    process: None ---> direction=direction ---> None
    """
    def __init__(self, driver, timedelay):
        super(FollowLaneEvent, self).__init__(driver)
        self.timedelay = timedelay
        self.direction = 50
        self.direct_cache = np.full((10, ), 50, dtype=int)
        self.loop_idx = loop_idx(10)
        self.idx = 0
        self.controller = FollowLineCtr
        self.timer = threading.Thread(target = self.set_direct)
        self.timer.setDaemon(True)
        self.timer.start()  #在等红绿灯的时候就会改方向，可能需要调整

    def is_start(self):
        """ 事件是否开始 """
        return True

    def __del__(self):
        self.flag = False

    def is_end(self):
        """ 事件是否终止 """
        return False

    def strategy(self):
        """ 控制策略 """
        bias, slope = self.driver.get_lane()
        bias = -bias
        self.direction = int( self.controller.control(bias, slope) + 50 )
        # 限位
        # bias_sign = 1 if bias>=0 else -1
        # slope_sign = 1 if slope>=0 else -1
        # gear_direct = (0, 20, 50, 75, 100)

        #     if np.abs(bias) >= 19.5:
        #         bias = bias_sign * 19.5
        #     gear = int((bias + 19.5) / 8)
        #     self.direction = gear_direct[gear]
        #     return
        # if np.abs(slope) > 1.5:
        #     self.direction = slope_sign * 50 + 50
        #     return

    def set_direct(self):
        while True:
            self.idx = self.loop_idx.next()
            self.driver.set_direction(self.direct_cache[self.idx])
            self.direct_cache[self.idx] = self.direction
            time.sleep(0.1)


class FollowLidarEvent(DriverEvent):
    '''
    雷达导航
    is_start: 雷达检测到挡板
    is_end: 雷达没有检测到挡板
    strategy: 分档位，根据距离和角度偏差的不同设定方向
    process: None ---> direction=direction ---> None
    '''
    def __init__(self, driver):
        super(FollowLidarEvent, self).__init__(driver)

    def is_start(self):
        _, board, _, _ = self.driver.get_lidar()
        if board:
            return True
        return False

    def is_end(self):
        if not self.is_start():
            return True
        return False

    def strategy(self):
        # bias为正靠右，angle为正朝右
        _, _, bias, angle = self.driver.get_lidar()
        norm = bias / 15 + angle / 25  # 归一化
        sign_norm = 1 if norm > 0 else -1
        if abs(norm) >= 1:
            norm = sign_norm
        self.driver.set_direction(50 - norm * 50)
        return True


class CrossBridgeEvent(DriverEvent):
    '''
    过桥事件
    is_start: 满足以下条件：
            (1)检测到挡板
            (2)俯仰角大于一定正向角度
    is_end: 满足以下条件：
            (1)没有检测到挡板
            (2)俯仰角大于一定负向角度
    strategy: 上桥加速，下桥减速
    process: None ---> speed=speed ---> None
    '''
    def __init__(self, driver, imu_limit, speed_upper, speed_normal, speed_limit):
        super(CrossBridgeEvent, self).__init__(driver)
        self.imu_limit = imu_limit
        self.speed_upper = speed_upper
        self.speed_normal = speed_normal
        self.speed_limit = speed_limit

    def is_start(self):
        _, board, _, _ = self.driver.get_lidar()
        imu, _, _ = self.driver.get_theta()
        if board and imu > self.imu_limit:
            return True
        return False

    def is_end(self):
        _, board, _, _ = self.driver.get_lidar()
        imu, _, _ = self.driver.get_theta()
        if not board and imu > -self.imu_limit:
            return True
        return False

    def strategy(self):
        _, _, bias, angle = self.driver.get_lidar()
        norm = bias / 15 + angle / 25  # 归一化
        imu, _, _ = self.driver.get_theta()
        sign_norm = 1 if norm > 0 else -1
        if abs(norm) >= 1:
            norm = sign_norm
        self.driver.set_direction(50 - norm * 50)
        if imu > self.imu_limit:
            self.driver.set_speed(self.speed_upper)
        elif self.driver.get_speed() < self.speed_limit:
            self.driver.set_mode('D')
            self.driver.set_speed(self.speed_normal)
        else:
            self.driver.set_mode('N')
        return True


class ObstacleEvent(DriverEvent):
    '''
    障碍物事件
    is_start: 检测到障碍物
    is_end: 没有检测到障碍物
    strategy: 停下
    process: None ---> speed=0&mode='N' ---> mode='D'&speed=speed
    '''
    def __init__(self, driver, speed_normal):
        super(ObstacleEvent, self).__init__(driver)
        self.speed_normal = speed_normal

    def is_start(self):
        obstacle, _, _, _ = self.driver.get_lidar()
        if obstacle:
            return True
        return False

    def is_end(self):
        if not self.is_start():
            self.driver.set_mode('D')
            self.driver.set_speed(self.speed_normal)
            return True
        return False

    def strategy(self):
        self.driver.set_speed(0)
        if self.driver.get_speed() <= 2:
            self.driver.set_mode('N')
        return True


class RedStopEvent(DriverEvent):
    '''
    红灯策略
    is_start: 目标检测到红灯，四个条件需要同时满足：
             (1)box面积大于0.1w*0.1h
             (2)红灯label的score>0.9
             (3)红灯位于图片的上方，即y_max<0.2h
             (4)连续1个输出满足上述要求
    is_end: is_start条件任意一个不满足则is_end，档位调为D
    strategy: 直接刹车速度为0,速度小于2时档位调为P
    process: None ---> speed=0&mode='N' ---> mode='D'
    '''
    def __init__(self, driver, scale_prop, y_limit, score_limit=0.5):
        """
        初始化
        :param area_thr: 检测红灯的面积阈值
        :param score_thr: 检测红灯的置信度阈值
        :param y_thr: 红灯高度阈值
        """
        super(RedStopEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit

    def is_start(self):
        """ 事件是否开始 """
        width = 1280
        height = 720
        flag, x_min, y_min, x_max, y_max, score = self.driver.get_objs(2)
        if flag and score > self.score_limit:
            area = (x_max - x_min) * (y_max - y_min)
            scale = area / (self.scale_prop * width * height)
            if scale >= 1 and y_max <= self.y_limit * height:
                return True
        return False

    def is_end(self):
        """ 事件是否终止 """
        if not self.is_start():
            self.driver.set_mode('D')
            return True
        return False

    def strategy(self):
        """ 控制策略 """
        self.driver.set_speed(0)
        if self.driver.get_speed() <= 2:
            self.driver.set_mode('N')


class GreenGoEvent(DriverEvent):
    """
    红灯策略
    is_start: 目标检测到绿灯，四个条件需要同时满足：
             (1)box面积大于0.1w*0.1h
             (2)绿灯label的score>0.9
             (3)绿灯位于图片的上方，即y_max<0.2h
             (4)连续1个输出满足上述要求
             档位调为D
    is_end: is_start条件任意一个不满足则is_end
    strategy: 直接刹车速度为0
    process: mode='D' ---> speed=speed ---> None
    """
    def __init__(self, driver, scale_prop, y_limit, speed, score_limit=0.5):
        """
        初始化
        :param area_thr: 检测红灯的面积阈值
        :param score_thr: 检测红灯的置信度阈值
        :param y_thr: 红灯高度阈值
        """
        super(GreenGoEvent, self).__init__(driver)
        self.speed = speed
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit

    def is_start(self):
        """ 事件是否开始 """
        width = 1280
        height = 720
        flag, x_min, y_min, x_max, y_max, score = self.driver.get_objs(0)
        if flag and score > self.score_limit:
            area = (x_max - x_min) * (y_max - y_min)
            scale = area / (self.scale_prop * width * height)
            if scale >= 1 and y_max <= self.y_limit * height:
                self.driver.set_mode('D')
                return True
        return False

    def is_end(self):
        """ 事件是否终止 """
        return not self.is_start()

    def strategy(self):
        """ 控制策略 """
        self.driver.set_speed(self.speed)

#"labels_list": ["green_go", "pedestrian_crossing", "red_stop", "speed_limited", "speed_minimum", "speed_unlimited", "yellow_back"]


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
    process: None ---> speed=0&mode='N' ---> mode='D',speed=speed
    '''
    def __init__(self, driver, scale_prop, y_limit, speed_normal, score_limit=0.5):
        super(PedestrianEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit
        self.speed_normal = speed_normal
        self.time = time.time()

    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(1)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag and (score >= self.score_limit) and (scale >= 1) and (y_min >= self.y_limit * height):
            return True
        return False

    def is_end(self):
        if self.driver.get_speed() == 0:
            self.driver.set_mode('D')
            self.driver.set_speed(self.speed_normal)
            return True
        return False

    def strategy(self):
        self.driver.set_speed(0)
        if self.driver.get_speed() <= 2:
            self.driver.set_mode('N')


class SpeedLimitedEvent(DriverEvent):
    '''
    区间限速策略
    is_start: 目标检测到限速标志，四个条件需要同时满足:
            (1)box面积大于0.15w*0.15h
            (2)限速标志label的score>0.9
            (3)限速标识位于图片的上方，即y_max<0.7h
            (4)连续1个输出满足上述要求
    is_end: 目标检测到解除限速标志，四个条件需要同时满足:
            (1)box面积大于0.15w*0.15h
            (2)解除限速标志label的score>0.9
            (3)解除限速标识位于图片的上方，即y_max<0.7h
            (4)连续1个输出满足上述要求
    strategy: 速度<=1km/h
    process: None ---> speed=speed_low --->speed=speed_normal
    '''
    def __init__(self, driver, scale_prop, y_limit, speed_low, speed_normal, score_limit=0.5):
        super(SpeedLimitedEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit
        self.speed_low = speed_low
        self.speed_normal = speed_normal

    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(3)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag and (score >= self.score_limit) and (scale >= 1) and (y_min <= self.y_limit * height):
            return True
        return False

    def is_end(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(5)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag and (score >= self.score_limit) and (scale >= 1) and (y_min <= self.y_limit * height):
            self.driver.set_speed(self.speed_normal)
            return True
        return False

    def strategy(self):
        self.driver.set_speed(self.speed_low)


class SpeedMinimumEvent(DriverEvent):
    '''
    路段测速策略
    is_start: 目标检测到最低速度标志，四个条件需要同时满足：
            (1)box面积大于0.15w*0.15h
            (2)限速标志label的score>0.9
            (3)限速标识位于图片的上方，即y_max<0.7h
            (4)连续1个输出满足上述要求
    is_end: is_start任意条件不满足
    strategy: 速度设置为30
    process: None ---> speed=speed_high --->speed=speed_normal
    '''
    def __init__(self, driver, scale_prop, y_limit, speed_high, speed_normal, score_limit):
        super(SpeedMinimumEvent, self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit
        self.speed_high = speed_high
        self.speed_normal = speed_normal

    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(4)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag and (score >= self.score_limit) and (scale >= 1) and (y_min <= self.y_limit * height):
            return True
        return False

    def is_end(self):
        if not self.is_start():
            self.driver.set_speed(self.speed_normal)
            return True
        return False

    def strategy(self):
        self.driver.set_speed(self.speed_high)


class YellowBackEvent(DriverEvent):
    '''
    倒车策略
    is_start: 目标检测到黄灯，四个条件需要同时满足：
             (1)box面积大于0.1w*0.1h
             (2)黄灯label的score>0.9
             (3)黄灯位于图片的上方，即y_max<0.2h
             (4)连续1个输出满足上述要求
             档位调为R
    is_end: 超声波距离过近
    strategy: 先向左转弯再向右转弯再直线倒车，转弯事件待调
    process: mode='N' ---> mode='R'&direction=direction&speed=speed ---> speed=0
    '''
    def __init__(self, driver, scale_prop, y_limit, speed, score_limit, range_limit, turn_time, back_direction):
        super(YellowBackEvent,self).__init__(driver)
        self.scale_prop = scale_prop
        self.score_limit = score_limit
        self.y_limit = y_limit
        self.speed = speed
        self.range_limit = range_limit
        self.turn_time = turn_time
        self.back_direction = back_direction
        self.phase = 1
        self.timer1 = threading.Timer(self.turn_time, self.delay1)
        self.timer2 = threading.Timer(self.turn_time, self.delay2)
        self.timer1.setDaemon(True)
        self.timer2.setDaemon(True)


    def is_start(self):
        width = 1280
        height = 720
        flag, x_min, x_max, y_min, y_max, score = self.driver.get_objs(6)
        scale = (y_max - y_min) * (x_max - x_min) / (self.scale_prop * width * height)
        if flag and (score >= self.score_limit) and (scale >= 1) and (y_min <= self.y_limit * height):
            self.driver.set_mode('N')
            self.timer1.start()
            return True
        return False

    def is_end(self):
        supersonic = self.driver.get_supersonic()
        if supersonic < self.range_limit:
            self.driver.set_speed(0)
            self.driver.set_mode('P')
            return True
        return False

    def strategy(self):
        self.driver.set_mode('R')
        if self.phase == 1:
            self.driver.set_speed(self.speed)
            self.driver.set_direction(self.back_direction)
        elif self.phase == 2:
            self.driver.set_speed(self.speed)
            self.driver.set_direction(50 * 2 - self.back_direction)
        elif self.phase == 3:
            self.driver.set_speed(self.speed)
            self.driver.set_direction(50)
        return True

    def delay1(self):
        self.timer2.start()
        self.phase = 2

    def delay2(self):
        self.phase = 3


