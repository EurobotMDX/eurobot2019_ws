import cv2
import numpy as np

img1 = cv2.imread("red-puk.png", cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread("puks.jpg", cv2.IMREAD_GRAYSCALE)

#Orb Detector - can be different
orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# for d in des1:
#     print(d)
#
# Brute force matching
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True) #it varies the NORM
matches = bf.match(des1, des2)
matches = sorted(matches, key = lambda x:x.distance) #sort from lower distance between mathes to highest

matching_result = cv2.drawMatches(img1, kp1, img2, kp2, matches[:20], None, flags = 2) #number of results varies

for m in matches:
    print(m.distance)
print("Number of matches: ", len(matches))

#cv2.imshow("img1", img1)
#cv2.imshow("img2", img2)
cv2.imshow("Matching result", matching_result)
cv2.waitKey(0)
cv2.destroyAllWindows()