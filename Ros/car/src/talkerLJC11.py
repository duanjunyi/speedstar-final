#!/usr/bin/env python
# license removed for brevity
import rospy
import socket
import time
from std_msgs.msg import Int32
from collections import deque
import numpy as np

cameraMatrix= np.array([[780.0872,0,674.2851],[0,923.9321,326.8382],[0,0,1]])
T2= np.array([[0.9538,0.044,0.2972,-184.5591],[0.2994,-0.0576,-0.9524,354.4698],[-0.0252,0.9973,-0.0683,982.8898],[0,0,0,1]])

def pixel2camera(T, mtx, u, v):
    Matrix = np.zeros([3, 4])
    for i in range(3):
        for j in range(3):
            Matrix[i][j] = mtx[i][j]
    M = np.matmul(Matrix, T)
    xw = (M[0][1] * M[1][3] - M[0][3] * M[1][1] + M[1][1] * M[2][3] * u - M[1][3] * M[2][1] * u - M[0][1] * M[2][3] * v + M[0][3] * M[2][1] * v) /\
         (M[0][0] * M[1][1] - M[0][1] * M[1][0] + M[1][0] * M[2][1] * u - M[1][1] * M[2][0] * u - M[0][0] * M[2][1] * v + M[0][1] * M[2][0] * v)
    yw = -(M[0][0] * M[1][3] - M[0][3] * M[1][0] + M[1][0] * M[2][3] * u - M[1][3] * M[2][0] * u - M[0][0] * M[2][3] * v + M[0][3] * M[2][0] * v) /\
         (M[0][0] * M[1][1] - M[0][1] * M[1][0] + M[1][0] * M[2][1] * u - M[1][1] * M[2][0] * u - M[0][0] * M[2][1] * v + M[0][1] * M[2][0] * v)
    worldPosition = np.array([[xw], [yw], [0], [1]])
    cameraPosition = np.matmul(T, worldPosition)
    return cameraPosition[0][0], cameraPosition[1][0], cameraPosition[2][0]



def talker():
    host='192.168.2.111'
    port=7777
      
    pub1 = rospy.Publisher('/hilens', Int32 , queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #the stage that control the start
    flag=0

    # initial queue for each condition(for situation in labeldict)
    red_stop_num=deque([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],maxlen=15)
    red_stop_num_nosee=deque([0,0,0,0,0,0,0,0,0,0],maxlen=10)
    
    green_go_num_start=deque([0,0,0,0,0,0,0],maxlen=7)
    pedestrian_crossing_num=deque([0,0,0,0,0,0,0],maxlen=7)
    speed_limited_num=deque([0,0,0,0,0,0,0],maxlen=7)
    speed_unlimited_num=deque([0,0,0,0,0,0,0],maxlen=7)
    

    yellow_back_num=deque([0,0,0,0,0,0,0],maxlen=7)
    yellow_back_noseenum=deque([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],maxlen=15)


    
    redflag=0

    speed_limited_flag=0
    speed_unlimited_flag=0

    yellowflag=0

    jiansuflag=0

    msg_hilens=100

    have_send_msg_hilens=[]
   


  
    while not rospy.is_shutdown():
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)  
        s.connect((host,port))      
        data=s.recv(1024)

        # the situation that don't see anything
        if len(data)==0:
            print('null')
            green_go_num_start.append(0)
            pedestrian_crossing_num.append(0)
            speed_limited_num.append(0)
            speed_unlimited_num.append(0)
            yellow_back_num.append(0)

            red_stop_num.append(0)

            if redflag==1:
                red_stop_num_nosee.append(1)

            if yellowflag==1 and redflag==0:
                yellow_back_noseenum.append(1)


        # the situation that see something
        if len(data)>0:
            data=eval(data)

            num=0
            numred0=0
            for key in data:
                print('{}--{}--{}--{}--{}--{}'.format(key,data[key][0],data[key][1],data[key][2],data[key][3],data[key][4]))
                labelname=key
                conf=data[key][0]
                xmin=data[key][1]
                xmax=data[key][2]
                ymin=data[key][3]
                ymax=data[key][4]
                area=abs(xmax-xmin)*abs(ymax-ymin)
                middlex=(xmax-xmin)*0.5
                middley=(ymax-ymin)*0.5
                x,y,z=pixel2camera(T2,cameraMatrix,middlex,middley)
                x1=x/10
                y1=y/10
                z1=z/10
                              
                if flag==0 and labelname=='green_go':
                    green_go_num_start.append(1)

                if labelname=='red_stop' and conf>0.8 and flag>0 and redflag==0:
                    red_stop_num.append(1)
                    red_stop_num.append(1)
                if labelname =='red_stop' and conf>0.5 and flag>0:
                    numred0+=1

                if labelname=='yellow_back' and conf>0.7 and yellowflag==0 and flag>0:
                    yellow_back_num.append(1)

                if labelname=='yellow_back' and flag>0:
                    yellow_back_noseenum.append(0)
                    num+=1

                if labelname=='speed_limited' and area>20000 and ymin<300 and speed_limited_flag==0:
                    speed_limited_num.append(1)

                if labelname=='speed_unlimited' and area>20000 and ymin<220 and speed_unlimited_flag==0:
                    speed_unlimited_num.append(1)

                if labelname=='pedestrian_crossing' and jiansuflag==1:
                    if y1<130 and (xmax-xmin)>600:
                        pedestrian_crossing_num.append(1)

                if labelname=='pedestrian_crossing' and (xmax-xmin)>350 and y1>300 and jiansuflag==0:
                    msg_hilens=9
                    have_send_msg_hilens.append(msg_hilens)
                    jiansuflag=1
                    for i in range(3):
                        pub1.publish(msg_hilens)                
                        rate.sleep()
                    s.close()
                    continue
            if numred0==0 and flag>0:
            	red_stop_num.append(0)
            	red_stop_num.append(0)


                    
            if numred0==0 and redflag==1:
                red_stop_num_nosee.append(1)


            if num==0 and  yellowflag==1 and numred==0 and redflag==0:
                yellow_back_noseenum.append(1)

           

        #send message to hilens
        if flag==0 and green_go_num_start.count(1)>2:
            flag+=1
            msg_hilens =0
            have_send_msg_hilens.append(msg_hilens)
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue

        if red_stop_num.count(1)>8 and redflag==0:
            red_stop_num=deque([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],maxlen=15)
            redflag=1
            have_send_msg_hilens.append(msg_hilens)
            msg_hilens =7
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue

        if red_stop_num_nosee.count(1)>4 and redflag==1:
            red_stop_num_nosee=deque([0,0,0,0,0,0,0,0,0,0],maxlen=10)
            redflag=0
            msg_hilens=8
            have_send_msg_hilens.append(msg_hilens)
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue


        if pedestrian_crossing_num.count(1)>1:
            pedestrian_crossing_num=deque([0,0,0,0,0,0,0],maxlen=7)
            jiansuflag=0
            msg_hilens =1
            have_send_msg_hilens.append(msg_hilens)
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue
       
        if speed_limited_num.count(1)>2:
            speed_limited_num=deque([0,0,0,0,0,0,0],maxlen=7)
            speed_limited_flag=1
            msg_hilens =2
            have_send_msg_hilens.append(msg_hilens)
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue


        if speed_unlimited_num.count(1)>2:
            speed_unlimited_num=deque([0,0,0,0,0,0,0],maxlen=7)
            speed_unlimited_flag=1
            msg_hilens =3
            have_send_msg_hilens.append(msg_hilens)
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue
  
        if yellow_back_num.count(1)>4 and yellowflag==0:
            yellowflag=1
            yellow_back_num=deque([0,0,0,0,0,0,0],maxlen=7)
            msg_hilens =5
            have_send_msg_hilens.append(msg_hilens)
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue

        if yellow_back_noseenum.count(1)>7:
            msg_hilens=6
            have_send_msg_hilens.append(msg_hilens)
            for i in range(4):
                pub1.publish(msg_hilens)                
                rate.sleep()
            s.close()
            continue

        s.close()
        
        print('the msg_hilens that has send:')
        for i in range(len(have_send_msg_hilens)):
            print(have_send_msg_hilens[i])


           

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass