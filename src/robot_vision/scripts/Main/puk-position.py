# USAGE
# python videostream_demo.py
# python videostream_demo.py --picamera 1

# import the necessary packages
from imutils.video import VideoStream
import datetime
import argparse
import imutils
import time
import cv2
import numpy as np

# # construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-p", "--picamera", type=int, default=-1,
# 	help="whether or not the Raspberry Pi camera should be used")
# args = vars(ap.parse_args())

# def nothing(x):
#     #any operation with change trackbar
#     pass

# cv2.namedWindow("Trackbars")

# cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
# cv2.createTrackbar("L - S", "Trackbars", 130, 255, nothing)
# cv2.createTrackbar("L - V", "Trackbars", 255, 255, nothing)
# cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
# cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
# cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

# font = cv2.FONT_HERSHEY_COMPLEX


# # initialize the video stream and allow the cammera sensor to warmup
# vs = VideoStream(usePiCamera=args["picamera"] > 0).start()
# time.sleep(2.0)

RESCALE_FACTOR = 0.5
vs = cv2.VideoCapture(0)

vs.set(3, int(1280*RESCALE_FACTOR))
vs.set(4, int(720*RESCALE_FACTOR))

#
for i in range(15):
    _, frame = vs.read()

r = cv2.selectROI(frame)
print(r)
#time.sleep(555)
# loop over the frames from the video stream
while True:
	_, frame = vs.read()
	# grab the frame from the threaded video stream and resize it
	# # to have a maximum width of 400 pixels
	# frame = vs.read()
	# frame = imutils.resize(frame, width=400)
	img_crop = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
	hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)

	

    # definig the range of red color
	red_lower = np.array([0, 130, 250])
	red_upper = np.array([180, 255, 255])

    # defining the Range of Blue color
	blue_lower = np.array([60, 123, 120])
	blue_upper = np.array([118, 255, 255])

    # defining the Range of green color TODO: change to green
	green_lower = np.array([28, 144, 203])
	green_upper = np.array([180, 255, 255])

    # finding the range of red,blue and green color in the image
	red_mask = cv2.inRange(hsv, red_lower, red_upper)
	blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
	green_mask = cv2.inRange(hsv, green_lower, green_upper)

	# Morphological transformation, Dilation
    # kernal = np.ones((5, 5), "uint8")

    # red_mask = cv2.dilate(red_mask, kernal)
	res = cv2.bitwise_and(img_crop, img_crop, mask=red_mask)

    # blue_mask = cv2.dilate(blue_mask, kernal)
	#res1 = cv2.bitwise_and(frame, frame, mask=blue_mask)

    # green_mask = cv2.dilate(green_mask, kernal)
	#res2 = cv2.bitwise_and(frame, frame, mask=green_mask)

	
	# kernel = np.ones((15,15),np.float32)/225
	# smoothed = cv2.filter2D(res,-1,kernel)
	# #smoothed = cv2.filter2D(res1,-1,kernel)
	# #smoothed = cv2.filter2D(res2,-1,kernel)
	# cv2.imshow('Original',frame)
	# cv2.imshow('Averaging',smoothed)

	blur = cv2.GaussianBlur(res, (15, 15), 0)
	cv2.imshow('Gaussian Blurring', blur)

	_, puck = cv2.threshold(blur, 70, 255, cv2.THRESH_BINARY)
	cv2.imshow('Puck',puck)

	xs = np.where(puck != 0)[1]
	ys = np.where(puck != 0)[0]

	xstd = np.std(xs)
	ystd = np.std(ys)

	x_init_avg = np.mean(xs)
	y_init_avg = np.mean(ys)

	xs = [x for x in xs if x <= x_init_avg+xstd or x >= x_init_avg-xstd]
	ys = [y for y in ys if y <= y_init_avg+xstd or y >= y_init_avg-xstd]
	xavg = np.mean(xs)
	yavg = np.mean(ys)

	print(xavg, yavg)


	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()