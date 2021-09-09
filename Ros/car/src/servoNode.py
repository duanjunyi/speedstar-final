#!/usr/bin/env python
#coding=utf-8
import rospy
#倒入自定义的数据类型
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import threading
import math

# GLOBAL VARIABLES
lane_vel = Twist()
#direction_board = Twist()


direct_board = 0.0
direct_lane = 0.0
direct_laser = 0.0

flag_lane = 0
flag_board = 0
flag_laser = 0
flag_cross =0
flag_cross_tmp = 0

hilens_data = Int32()

angularScale = 9      # 180/30
servodata=50

angle = 50
speed = 26
speed_v = 0.0
path_length = 0.0
error = 0.0
error_tmp = 0.0
error_sum = 0.0

flag_start = -1
flag_yellow_back = -1
flag_stop = -1
flag_speed_limited = -1
flag_speed_unlimited = -1
flag_cross = -1
flag_cross_pds =-1


def thread_job():
    rospy.spin()
def speedcallback(msg):
    global path_length,speed_v,speed
    global error,error_tmp,error_sum
    if 0 <= msg.data <= 7000:
        Kp = 1.5
	if 10<speed<16:
	    Kp=2#1
	if speed <=10:
	    Kp = 1
        Ki = 0.0
        Kd = 0.2
        start_time1 = time.time()
        v = (msg.data*math.pi*0.175)/(36.48*60) 
        path_length += v*(time.time()-start_time1)
        error_tmp = error
        error = (speed*100*math.pi*0.175)/(36.48*60) - v
        error_sum = error_sum + error
        speed_v1 = v + Kp*error + Ki*error_sum + Kd*(error-error_tmp)
        speed_v = int(speed_v1*36.48*60/(math.pi*0.175*100))
        speed_v = min(max(0,speed_v),40)
        #print("the length of path:  ",path_length)


# for lane information
def flag_lane_callback(msg):
    global flag_lane
    flag_lane = msg.data
def  lanecallback(msg):
    global angularScale,flag_lane
    _servoCmdMsg = msg.angular.z * angularScale + 90
    flag_lane = msg.angular.x

    global direct_lane
    direct_lane = min(max(0, _servoCmdMsg), 180)
    direct_lane = 100-direct_lane*100/180
    #print ("received_lane:",msg.angular.x," flag_lane  ",direct_lane)


def laser_direction_callback(msg):
    global direct_laser,angularScale,flag_laser
    flag_laser = msg.angular.x

    _servoCmdMsg = msg.angular.z * angularScale + 90  
    direct_laser = min(max(0, _servoCmdMsg), 180)
    direct_laser = 100-direct_laser*100/180
    #print("flag_laser",flag_laser,"   ",direct_laser)
def flag_laser_callback(msg):
    global flag_laser
    flag_laser = msg.data


def hilens_callback(msg):
    global hilens_data,flag_start,flag_yellow_back,flag_cross, flag_stop,flag_speed_limited,flag_speed_unlimited 
    hilens_data = msg.data
    #print(msg.data)
    if hilens_data == 0:
        flag_start = 0
    if hilens_data == 5:
        flag_yellow_back = 0

    if hilens_data == 6:
        flag_stop = 0  

    if hilens_data == 2:
        flag_speed_limited = 0
    if hilens_data == 3:
        flag_speed_unlimited = 0
    if hilens_data == 1 and flag_cross ==-1:
	flag_cross = 1
    print("hilens",hilens_data,"**************************************************")
    
#rospy.loginfo(rospy.get_caller_id() + "traffic_light_data is %s", traffic_light_data)


# for board information
def flag_board_callback(msg):
    global flag_board
    flag_board = msg.data
def board_direction_callback(msg):
    global  direct_board,angularScale,flag_board
    direct_board_tmp = msg.angular.z * angularScale + 90
    direct_board = min(max(0, direct_board_tmp), 180)
    direct_board = 100-direct_board*100/180
    flag_board = msg.angular.x
    #print("flag_board",flag_board,"board",direct_board)

def flag_cross_callback(msg):
    global flag_cross_pds,flag_cross_tmp
    flag_cross_tmp = flag_cross
    flag_cross_pds = msg.data



def kinematicCtrl():
    
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    global speed,speed_v,servodata,flag_cross
    global direct_board,direct_lane, direct_laser,flag_lane, flag_board, flag_laser,hilens_data ,flag_cross_pds
    global flag_cross,flag_cross_tmp,flag_start,flag_yellow_back
    global flag_stop,flag_speed_limited,flag_speed_unlimited 
    pub1 = rospy.Publisher('/bluetooth/received/manual', Int32 , queue_size=10)
    pub2 = rospy.Publisher('/auto_driver/send/direction', Int32 , queue_size=10)
    pub3 = rospy.Publisher('/auto_driver/send/speed', Int32 , queue_size=10)
    pub4 = rospy.Publisher('/auto_driver/send/gear', Int32 , queue_size=10)
    pub5 = rospy.Publisher('/flag_final', Int32 , queue_size=10)
    
    manul=0       # 0 - Automatic(自动); 1 - Manual (手动操控)
   # speed=20      # SPEED (0～100之间的值)
    direction=50  # 0-LEFT-50-RIGHT-100 (0-49:左转，50:直行，51～100:右转)
    gear=1        # 1 - DRIVE, 2 - NEUTRAL, 3 - PARK, 4 - REVERSE
                  # 1:前进挡 2:空挡 3:停车挡 4:倒挡

    cmd_vel = Twist()
    flag=0
    p_flag=1
    servodata_list=[]
    n_loop=1
    
    rospy.init_node('kinematicCtrl', anonymous=True)
    
    add_thread = threading.Thread(target = thread_job)
    
    add_thread.start()
    # subscirber 
    rate = rospy.Rate(10) # 8Hz
    rospy.Subscriber("lane_direction", Twist, lanecallback)
    rospy.Subscriber("flag_lane",Int32,flag_lane_callback)

    rospy.Subscriber("/hilens", Int32, hilens_callback)

    rospy.Subscriber("flag_laser_board", Int32, flag_board_callback) # flag board
    rospy.Subscriber("board_direction", Twist, board_direction_callback)

    rospy.Subscriber("flag_cross", Int32, flag_cross_callback)

    rospy.Subscriber("laser_direction",Twist,laser_direction_callback)
    rospy.Subscriber("flag_laser",Int32,flag_laser_callback)

    rospy.Subscriber("/vcu/ActualMotorSpeed", Int32, speedcallback)
    #rospy.Subscriber("/vcu/ActualMotorSpeed", Int32, speedcallback)
    
    #更新频率是1hz
    rospy.loginfo(rospy.is_shutdown())
    n = 1
    servodata_list = n * [servodata]
    state_speed = 0
    state_direction = 0 
    state_cross = 0
    start_time = 0.0
    start_time_green = 0.0
    end_time = 0.0
    state_end = 0
    state_speed_limited = 0
    state_speed_unlimited = 0
    laser_pre = 0
    #time.sleep(3)
    while not rospy.is_shutdown():
        global j
        global angle
    	flag_final = 0
        # KINEMATIC CONTROL CODE HERE
        test_time = time.time()
        if   flag_laser == 1.0 :
            servodata = direct_laser


        else:

 
            if flag_lane == 1: 
                servodata = direct_lane 
        #if flag_lane == 0 and flag_laser == 0:



         #   servodata = 50


        if flag_stop == 0 and state_end == 0:
            end_time = time.time()
            state_end = 1
        if (time.time() - end_time )> 1.2 and state_end == 1: # change time              
            while 1:
                pub2.publish(servodata)
                speed = 0
                pub3.publish(speed_v)
                pub4.publish(3)
                time.sleep(0.1) 
        laser_pre = flag_laser

        if flag_yellow_back == 0 and flag_board == 1 :#chang_yellow 
            servodata = direct_board 




       
      
        if hilens_data == 1 or hilens_data == 4: 
            start_time = time.time()
            state_speed = 1
            hilens_data = 100
            #for  i in range(10):
             #   pub2.publish(servodata)
              #  speed = 25
               # pub3.publish(speed_v)
                #pub4.publish(1)
                #rate.sleep()
            #time.sleep(2)
           
        if   state_speed == 1 : # change time_cross
            for  i in range(5):
	        
                pub2.publish(servodata)
                speed = 26
                pub3.publish(speed_v)
                pub4.publish(3)
                rate.sleep()
            time.sleep(0.1)
            state_speed =2
	    #exit()
        
            #if flag_cross == 1 and flag_cross_tmp == 1:

            

     

        direction = min(max(0,servodata),100)

        #print(direction)

        if  flag_start == 0:
            speed = 26

            if flag_speed_limited == 0 and flag_speed_unlimited == -1:
                speed = 10
            if flag_speed_unlimited == 0:
                speed = 26
	    if hilens_data==9 or hilens_data==10:
	        speed = 10
            #print(flag_start)
            start_time_green = time.time()
            while time.time() - start_time_green < 2.5 and state_direction ==0:#2.5
                pub2.publish(50)
                pub3.publish(speed_v)
                pub4.publish(1)
                time.sleep(0.1)
            state_direction = 1
	    gear = 1
            while  state_speed ==2 and flag_cross_pds == 0:
		for i in range(1):
                    pub2.publish(servodata)
                    #speed = 0
                    pub3.publish(speed_v)
		    #gear = 3
                    pub4.publish(3)
                    time.sleep(0.1)
		if flag_cross_pds == 1:
		    flag_final = 1 

            if flag_yellow_back == 0 and flag_board == 1 :#chang_yellow 
                servodata = direct_board 
	        speed = 15
		
	    state_speed =3
		#flag_cross_pds =  1
	    if hilens_data==7:
		gear = 3
            pub2.publish(direction)
            #print ("*****,direction",direction)
            #print("***speed",speed_v)

            pub3.publish(speed_v)
            pub4.publish(gear)
	    pub5.publish(flag_final)
            angle = direction
            rate.sleep()
	    print("test cost time: ", time.time()-test_time)
        else:
            speed = 0
            pub2.publish(49)
            pub3.publish(0)
            pub4.publish(3)
            time.sleep(0.1)


if __name__ == '__main__':
    kinematicCtrl()

