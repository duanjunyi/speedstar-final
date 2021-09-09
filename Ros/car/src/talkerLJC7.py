#!/usr/bin/env python
# license removed for brevity

import socket
import time
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
    temp=0
 

    while True:
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)  
        s.connect((host,port))      
        data=s.recv(1024)



        # the situation that don't see anything
        if len(data)==0:
            print('null')

        # the situation that see something
        if len(data)>0:
            data=eval(data)
            for key in data:
                print('{}--{}--{}--{}--{}--{}'.format(key,data[key][0],data[key][1],data[key][2],data[key][3],data[key][4]))
                if key=='pedestrian_crossing':
                    area=(data[key][2]-data[key][1])*(data[key][4]-data[key][3])
                    print('area--{}'.format(area))
                    print('xmax-xmin--{}'.format(data[key][2]-data[key][1]))
                    print('xmax+xmin--{}'.format((data[key][2]+data[key][1])/2))
                    
                    print('ymin--{}'.format(data[key][3]))
                    print('ymax--{}'.format(data[key][4]))
                    
                    middlex= (data[key][2]-data[key][1])*0.5
                    middley= (data[key][4]-data[key][3])*0.5
                    x,y,z=pixel2camera(T2,cameraMatrix,middlex,middley)
                    print('x--{}cm   y--{}cm   z--{}cm'.format(x/10,y/10,z/10))
                    
                    
            print('\n')
            time.sleep(0.1)
              

        s.close()
        
        
           

if __name__ == '__main__':
   
        talker()
    
