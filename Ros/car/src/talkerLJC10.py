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

    start_time=time.time()+500
  

    # global flag 
    flag=0

    '''
    labeldict for each flag
    '''
    labeldict={0:'green_go',1:'pedestrian_crossing',2:'speed_limited',3:'speed_unlimited',4:'pedestrian_crossing',5:'yellow_back',6:'yellow_back',7:'yellow_back'}
    
    # initial queue for each condition(for situation in labeldict)
    green_go_num=deque([0,0,0,0,0,0,0],maxlen=7)
    pedestrian_crossing_num1=deque([0,0,0,0,0,0,0],maxlen=7)
    speed_limited_num=deque([0,0,0,0,0,0,0],maxlen=7)
    speed_unlimited_num=deque([0,0,0,0,0,0,0],maxlen=7)
    pedestrian_crossing_num2=deque([0,0,0,0,0,0,0],maxlen=7)

  

    yellow_back_num=deque([0,0,0,0,0,0,0,0,0,0],maxlen=10)
    yellow_back_noseenum=deque([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],maxlen=15)

    yellowflag=0

    jiansu1flag=0
    jiansu2flag=0
    msg_hilens=100


  
    while not rospy.is_shutdown():
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)  
        s.connect((host,port))      
        data=s.recv(1024)


        # the situation that don't see anything
        if len(data)==0:
            print('null')
            if flag==0:
                green_go_num.append(0)
            if flag==1:
                pedestrian_crossing_num1.append(0)
            if flag==2:
                speed_limited_num.append(0)
            if flag==3:
                speed_unlimited_num.append(0)
            if flag==4:
                pedestrian_crossing_num1.append(0)

        
            #yellow_back_num is for special situation
            yellow_back_num.append(0)

            if yellowflag==1:
                yellow_back_noseenum.append(1)



        # the situation that see something
        if len(data)>0:
            data=eval(data)
            labelname=" "
            conf=0
            xmin=0
            xmax=0
            ymin=0
            ymax=0
            area=0
            satisfied=0

            for key in data:
                if key == labeldict[flag]:
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
                    print('xmax-xmin--{}'.format(xmax-xmin))
                    print('x--{}  y--{}  z--{}'.format(x1,y1,z1))
                    
                    satisfied=1
                    break
                                                               
            if satisfied==1:

               
                if flag==0 :
                    green_go_num.append(1)
                if flag==1  and jiansu1flag==0:
 
                    msg_hilens=9
                    jiansu1flag=1
                    for i in range(3):
                        print('limit speed')
                        pub1.publish(msg_hilens)                
                        rate.sleep()
                    s.close()
                    continue
                    
                if flag==1 and jiansu1flag==1:
                    if y1<125 and (xmax-xmin)>500:
                        pedestrian_crossing_num1.append(1)
                    
 
                if flag==2  and area>20000 and ymin<300:
                    speed_limited_num.append(1)
                if flag==3  and area>20000 and ymin<250:
                    speed_unlimited_num.append(1)

                if flag==4  and jiansu2flag==0:

                    msg_hilens=10
                    jiansu2flag=1
                    for i in range(3):
                        print('limit speed')
                        pub1.publish(msg_hilens)                
                        rate.sleep()
                    s.close()
                    continue

                if flag==4 and jiansu2flag==1:
                   
                    
                    if y1<125 and (xmax-xmin)>500:
                        pedestrian_crossing_num2.append(1)
                   
                if flag==5:
                    yellow_back_num.append(1)
                if flag==6:
                    yellow_back_noseenum.append(0)

            else:
                print('no staisfied label')
                # the situation that don't see the corresponding thing
                if flag==0:
                    green_go_num.append(0)
                if flag==1:
                    pedestrian_crossing_num1.append(0)

                if flag==2:
                    speed_limited_num.append(0)
                if flag==3:
                    speed_unlimited_num.append(0)
                if flag==4:
                    pedestrian_crossing_num2.append(0)

                if flag==6:
                    yellow_back_noseenum.append(1)


        #change flag and send message to hilens

        if flag==0 and green_go_num.count(1)>1:
            flag+=1
            msg_hilens =0
            for i in range(10):
                pub1.publish(msg_hilens)                
                rate.sleep()
            start_time=time.time()

        if time.time()-start_time>220 and flag==1:
            flag+=1
            for i in range(10):
                print('change flag because surpass time')
 
        if flag==1 and pedestrian_crossing_num1.count(1)>1:
            flag+=1
      
            msg_hilens =1
            for i in range(10):
                pub1.publish(msg_hilens)                
                rate.sleep()
            time.sleep(4)
            
       
        if flag==2 and speed_limited_num.count(1)>2:
            flag+=1
            
            msg_hilens =2
            for i in range(10):
                pub1.publish(msg_hilens)                
                rate.sleep()

        if flag==3 and speed_unlimited_num.count(1)>2:
            flag+=1
            msg_hilens =3
            for i in range(10):
                pub1.publish(msg_hilens)                
                rate.sleep()
            time.sleep(10)

        if flag==4 and pedestrian_crossing_num2.count(1)>1:
            flag+=1
            msg_hilens =4
            for i in range(10):
                pub1.publish(msg_hilens)                
                rate.sleep()



        if flag==5 and yellow_back_num.count(1)>5:
            flag+=1
            yellowflag=1
            msg_hilens =5
            yellow_back_num=deque([0,0,0,0,0,0,0,0,0,0],maxlen=10)
	
            for i in range(10):
                pub1.publish(msg_hilens)                
                rate.sleep()
            time.sleep(2)

        if flag==6 and yellow_back_noseenum.count(1)>7:
            flag+=1
            msg_hilens=6
            for i in range(10):
                pub1.publish(msg_hilens)                
                rate.sleep()

    
        s.close()
        print('flag={}'.format(flag))
        print('msg_hilens--{}'.format(msg_hilens))

           

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
