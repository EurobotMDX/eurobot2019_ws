#Not working - works partially
import cv2
#import cv2.cv as cv
import numpy as np


cap = cv2.VideoCapture(0)

while True:
    f ,img = cap.read()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_CUBIC)
    img = cv2.medianBlur(gray,1)
    #circles = cv2.HoughCircles(gray, cv2.CV_HOUGH_GRADIENT, 1, 100, param1=100, param2=1)
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
                               param1=50,param2=22,minRadius=15,maxRadius=18)

    if circles!=None:
        i = np.uint16(np.around(circles))
        cv2.circle(gray,(i[0,0,0],i[0,0,1]),i[0,0,2],(255,255,255),1)
        cv2.circle(gray,(i[0,0,0],i[0,0,1]),1,(255,255,255),1)
        center_x.append(i[0,0,0])
        center_y.append(i[0,0,1])

    cv2.imshow('detected circles',gray) #img
    cv2.waitKey(1)

cv2.destroyAllWindows()