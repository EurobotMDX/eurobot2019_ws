#Convert to video cap
import numpy as np
import cv2 as cv
img = cv.imread('blue-puk.png',0)
img = cv.medianBlur(img,5)
cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
                            param1=50,param2=52,minRadius=1,maxRadius=150)

circles = np.uint16(np.around(circles))
font = cv.FONT_HERSHEY_SIMPLEX
height, width = cimg.shape[:2]   
for i in circles[0,:]:
    # draw the outer circle
    cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
    cv.putText(cimg,'Center[x y radius]: ' + str(i),(i[0]+10,i[1]+i[2]+10), font, 0.5, (200,255,155), 1, cv.LINE_AA)



cv.imshow('detected circles',cimg)
cv.waitKey(0)
cv.destroyAllWindows()