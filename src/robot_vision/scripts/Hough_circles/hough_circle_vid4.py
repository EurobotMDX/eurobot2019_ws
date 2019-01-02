import cv2
import numpy as np

#cap = cv2.VideoCapture('/home/sm/Desktop/videos/VIRB0031.MP4')
cap = cv2.VideoCapture(0)

while True:
    ret, img = cap.read()
    cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,1,20,
                               param1=50,param2=22,minRadius=15,maxRadius=18)

    try:
        l = len(circles[0,:])
        for i in circles[0,:]:
            cv2.circle(img,(i[0],i[1]),15,(0,0,255),5)
    except TypeError:
        pass

    cv2.imshow('detected circles',img)
    cv2.waitKey(1)

cv2.destroyAllWindows()