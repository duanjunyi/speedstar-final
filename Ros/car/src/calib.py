
import cv2
 
cap = cv2.VideoCapture('/dev/video10')		
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
		
 

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/home/output.avi',fourcc, 20.0, (1280, 720))
str1 = "/home/pi/Pictures2/7.png"
num = 1
flag = 0
 
while(cap.isOpened()):
    ret, frame = cap.read()				
    if ret==True:
        #frame = cv2.flip(frame,0)			
        out.write(frame)					
 
        #cv2.imshow('frame',frame)  		
        if flag%20 == 0:
            cv2.imwrite(str1, frame)
            num = num+1
	    flag=0
        flag = flag+1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break
 
cap.release() 
out.release()
cv2.destroyAllWindows()
