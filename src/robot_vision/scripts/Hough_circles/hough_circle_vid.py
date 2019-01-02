import cv2
import numpy as np
import sys
cap = cv2.VideoCapture(0)

while(True):
    gray = cv2.medianBlur(cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2GRAY),5)
    cirles=cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 10)# ret=[[Xpos,Ypos,Radius],...]
    
    if cirles!=None:
        print("Circle There !")

    cv2.imshow('video',gray)
    if cv2.waitKey(1)==27:# esc Key
        break

cap.release()
cv2.destroyAllWindows()