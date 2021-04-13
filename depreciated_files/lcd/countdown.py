import cv2
import time

#initialize camera
cap = cv2.VideoCapture(0)
width=480
height=320
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)#set image width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)#set image height
#define countdown parameters
font = cv2.FONT_HERSHEY_SIMPLEX
scale = 5
color = (0, 0, 255)
thickness = 10
 
while True:
    # Read and display each frame
    ret, frame = cap.read()
    cv2.imshow('camera view',frame)
    k = cv2.waitKey(125)
    # Specify the countdown
    
    # set the key for the countdown to begin
    if k == ord('q'):
        j = 30
        while j>=10: #display number every ten loops
            ret, frame = cap.read()
            if j%10 == 0:#count 3 ->2 -> 1
                cv2.putText(frame,str(j//10),(250,200),font, scale,color,thickness,cv2.LINE_AA)
            cv2.imshow('camera view',frame)
            cv2.waitKey(125)
            j = j-1
        else:
            break

cap.release()
cv2.destroyAllWindows()