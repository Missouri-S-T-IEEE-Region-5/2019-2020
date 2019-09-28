import cv2
import numpy as np

cap=cv2.VideoCapture(0)

while True:
    k=cv2.waitKey(20)
    if k==27:
        break
    
    _, frame=cap.read()
    
    hsv_frame=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([0,0,0]) #-- Lower range --
    upper = np.array([127,127,127]) #-- Upper range --
    mask = cv2.inRange(hsv_frame, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask= mask) #-- Contains pixels having the gray color--
    cv2.imshow('Result',res)

cap.release()
cv2.destroyAllWindows()
