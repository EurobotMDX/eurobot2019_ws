#%%
import cv2
import numpy as py
import matplotlib.pyplot as plt 
%matplotlib inline

image = 'coins.png'
img = cv2.imread(image, 1)
img_orig = img.copy()
img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
plt.rcParams["figure.figsize"] = (16, 9)
plt.imshow(img,cmap='gray')

img = cv2.GaussianBlur(img, (21,21), cv2.BORDER_DEFAULT)
plt.rcParams["figure.figsize"] = (16,9)
plt.imshow(img,cmap='gray')

all_circs = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, 0.9, 120, param1 = 50, param2 = 30, minRadius = 60, maxRadius = 360)
all_circs_rounded = np.uint16(np.around(all_circs))

print(all_circs_rounded)
print(all_circs_rounded.shape)
print('I have found ' + str(all_circs_rounded.shape[1]) + ' coins.')

count = 1
for i in all_circs_rounded[0, :]:
    cv2.circle(img_orig, (i[0], i[1], i[2], (50, 200, 200), 5))
    cv2.circle(img_orig, (i[0], i[1]), 2, (255, 0, 0), 3)
    cv2.putText(img_orig, "Coin " + str(count), (i[0]-70,i[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255,0,0), 2)
    count += 1

plt.rcParams["figure.figsize"] = (16,0)
plt.imshow(img_orig)
