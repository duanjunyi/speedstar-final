#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from driver_utils import Driver
from driver_event import *
import signal

def sigint_handler(signal, frame):
    print('Terminated!')
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)


def main():
    #--- 小车驱动
    driver = Driver()

    #--- 定义事件列表
    follow_lane_event = FollowLaneEvent(driver, 1)
    red_stop_event = RedStopEvent(driver, 0.01, 0.2, 0.9)
    green_go_event = GreenGoEvent(driver, 0.01, 0.2, 50, 0.9)
    pedestrian_event = PedestrianEvent(driver, 0.06, 0.6, 0.9)
    speed_limited_event = SpeedLimitedEvent(driver, 0.02, 0.7, 10, 0.9)
    speed_minimum_event = SpeedMinimumEvent(driver, 0.02, 0.7, 70, 0.9)
    event_list = [follow_lane_event, red_stop_event, green_go_event, pedestrian_event, speed_limited_event,
                  speed_minimum_event]

    #--- 主循环
    rate = rospy.Rate(100)  # 循环频率
    event_running = set()   # 集合保存正在运行的事件 TODO 后续可以改成优先级列表

    while not rospy.is_shutdown():
        # 查询从未开始变化为开始的事件
        for i, event in enumerate(event_list):
            if event.is_start() and not i in event_running:
                event_running.add(i)

        # 遍历执行正在运行事件的策略，并将结束事件删除
        for i in event_running:
            if event_list[i].is_end():
                event_running.remove(i)
            else:
                event_list[i].strategy()

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('driver', anonymous=True)
    main()