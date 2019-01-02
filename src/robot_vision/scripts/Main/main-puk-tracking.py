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

RESCALE_FACTOR = 1.0
vs = cv2.VideoCapture(0)


WIDTH = int(1280*RESCALE_FACTOR)
HEIGHT = int(720*RESCALE_FACTOR)
vs.set(3, WIDTH)
vs.set(4, HEIGHT)
#Static HSV for testing
H_min = 60
S_min = 123
V_min = 120
H_max = 118
S_max = 255
V_max = 255


for i in range(15):
    _, frame = vs.read()

r = cv2.selectROI(frame)
print(r)
#time.sleep(555)
# loop over the frames from the video stream

if RESCALE_FACTOR == 0.5:
    r = (66, 21, 560, 270)
elif RESCALE_FACTOR == 1.0:
    r = (104, 36, 1136, 557)


while True:
    _, frame = vs.read()
	# grab the frame from the threaded video stream and resize it
	# # to have a maximum width of 400 pixels
	# frame = vs.read()
	# frame = imutils.resize(frame, width=400)
    image2 = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    frame_to_thresh = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)

	
    frame_to_thresh = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)

    thresh = cv2.inRange(
        frame_to_thresh, (H_min, S_min, V_min), (H_max, S_max, V_max))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    
	# definig the range of red color
	# red_lower = np.array([0, 130, 250])
	# red_upper = np.array([180, 255, 255])

    # # defining the Range of Blue color
	# blue_lower = np.array([60, 123, 120])
	# blue_upper = np.array([118, 255, 255])

    # # defining the Range of green color TODO: change to green
	# green_lower = np.array([28, 144, 203])
	# green_upper = np.array([180, 255, 255])

    # # finding the range of red,blue and green color in the image
	# red_mask = cv2.inRange(hsv, red_lower, red_upper)
	# blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
	# green_mask = cv2.inRange(hsv, green_lower, green_upper)

	# Morphological transformation, Dilation
    # kernal = np.ones((5, 5), "uint8")

    # red_mask = cv2.dilate(red_mask, kernal)
	#res = cv2.bitwise_and(img_crop, img_crop, mask=red_mask)

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

	# blur = cv2.GaussianBlur(res, (15, 15), 0)
	# cv2.imshow('Gaussian Blurring', blur)

	# _, puck = cv2.threshold(blur, 70, 255, cv2.THRESH_BINARY)
	# cv2.imshow('Puck',puck)

	# xs = np.where(puck != 0)[1]
	# ys = np.where(puck != 0)[0]

	# xstd = np.std(xs)
	# ystd = np.std(ys)

	# x_init_avg = np.mean(xs)
	# y_init_avg = np.mean(ys)

	# xs = [x for x in xs if x <= x_init_avg+xstd or x >= x_init_avg-xstd]
	# ys = [y for y in ys if y <= y_init_avg+xstd or y >= y_init_avg-xstd]
	# xavg = np.mean(xs)
	# yavg = np.mean(ys)

	# print(xavg, yavg)
    if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
        if radius > (RESCALE_FACTOR*20) and radius < (RESCALE_FACTOR*200): #  adding the less than part too.
                #print(radius)
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
            cv2.circle(image2, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(image2, center, 3, (0, 0, 255), -1)
            cv2.putText(image2, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            cv2.putText(image2, "(" + str(center[0]) + "," + str(center[1]) + ")", (center[
                            0] + 10, center[1] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)


            cropped_height, cropped_width, channels = image2.shape
            puck_x = center[0]/cropped_width
            puck_y = center[1]/cropped_height

                # get the actual desired y position
                # desired_arm_y = (puck_y*(ARM_Y_MAX*2))-ARM_Y_MAX
                # print(desired_arm_y)

                # swift.set_position(y=round(desired_arm_y, 2), x=200, speed=30000)

            print(puck_x)

	# show the frame to our screen
    
    cv2.imshow("Original", frame)
	#cv2.imshow("Marked", image2)
    key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()