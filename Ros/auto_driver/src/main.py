#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
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
    driver.set_mode('D')
    #--- 定义事件列表
    follow_lane_event = FollowLaneEvent(driver, 1)
    red_stop_event = RedStopEvent(driver, 0.01, 0.2, 0.9)
    green_go_event = GreenGoEvent(driver, 0.01, 0.2, 20, 0.9)
    pedestrian_event = PedestrianEvent(driver, 0.06, 0.6, 0.9)
    speed_limited_event = SpeedLimitedEvent(driver, 0.02, 0.7, 10, 20, 0.9)
    speed_minimum_event = SpeedMinimumEvent(driver, 0.02, 0.7, 30, 20, 0.9)
    event_list = [red_stop_event, pedestrian_event, green_go_event, follow_lane_event, speed_limited_event,
                  speed_minimum_event]  # 默认为优先级排序，越靠前优先级越高

    #--- 主循环
    rate = rospy.Rate(100)  # 循环频率
    event_running = []   # 集合保存正在运行的事件

    while not rospy.is_shutdown():
        # 查询从未开始变化为开始的事件并加入到运行事件列表中
        for i, event in enumerate(event_list):
            if event.is_start() and not i in event_running:
                event_running.append(i)

        # 冒泡算法根据优先级决定运行策略顺序
        for i in range(1, len(event_running)):
            for j in range(0, len(event_running) - i):
                if event_running[j] > event_running[j+1]:
                    event_running[j], event_running[j+1] = event_running[j+1], event_running[j]

        # 遍历执行正在运行事件的策略，并将结束事件删除
        for i in event_running:
            if event_list[i].is_end():
                event_running.remove(i)
            else:
                if i in [1, 2]:
                    event_list[i].strategy()
                    break  # 红灯，斑马线阻塞运行
                event_list[i].strategy()

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('driver', anonymous=True)
    main()