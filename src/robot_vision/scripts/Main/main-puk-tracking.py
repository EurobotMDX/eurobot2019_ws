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

RESCALE_FACTOR = 1.0
vs = cv2.VideoCapture(0)

WIDTH = int(1280*RESCALE_FACTOR)
HEIGHT = int(720*RESCALE_FACTOR)
vs.set(3, WIDTH)
vs.set(4, HEIGHT)

# defining the Range of Blue color
blue_lower = np.array([60, 123, 120])
blue_upper = np.array([118, 255, 255])

for i in range(15):
    _, frame = vs.read()


if RESCALE_FACTOR == 0.5:
    r = (66, 21, 560, 270)
elif RESCALE_FACTOR == 1.0:
    r = (104, 36, 1136, 557)


while True:
    _, frame = vs.read()

    crop_frame = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    frame_to_thresh = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV)

    blue_thresh = cv2.inRange(
    frame_to_thresh, blue_lower, blue_upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(blue_thresh, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
## TODO: Add Hierarchy for more than 1 contour - tracking-mul-col-labels.py
    if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
        if radius > (RESCALE_FACTOR*20) and radius < (RESCALE_FACTOR*400): #  adding the less than part too. Skale of circle
                #print(radius)
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
            cv2.circle(crop_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(crop_frame, center, 3, (0, 0, 255), -1)
            cv2.putText(crop_frame, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            cv2.putText(crop_frame, "(" + str(center[0]) + "," + str(center[1]) + ")", (center[
                            0] + 10, center[1] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            cropped_height, cropped_width, channels = crop_frame.shape
            puck_x = center[0]/cropped_width
            puck_y = center[1]/cropped_height

            print(puck_x)

	# show the frame to our screen
    
    cv2.imshow("Original", frame)
	#cv2.imshow("Marked", crop_frame)
    key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()