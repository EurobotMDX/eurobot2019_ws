import cv2
import numpy as numpy

img = cv2.imread("red-puk.png", cv2.IMREAD_GRAYSCALE)


sift = cv2.xfeatures2d.SIFT_create()
#faster 
surf = cv2.xfeatures2d.SURF_create()

orb = cv2.ORB_create(nfeatures=1500) #Could be with no args

#Drawing key-points features dewtected with sift
#kp = sift.detect(img, None)
#img = cv2.drawKeypoints(img, kp, None)
keypoints, descriptors = orb.detectAndCompute(img, None)
img = cv2.drawKeypoints(img, keypoints, None)

cv2.imshow("Image", img)
cv2.waitKey(0)
cv2.destroyAllWindows
