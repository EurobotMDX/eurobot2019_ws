import cv2
import numpy as np

device = cv2.VideoCapture(0)

while True:
    ret, frame = device.read()
    # _,frame = device.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_range = np.array([110,50,50])
    upper_range = np.array([130,255,255])

    # Blue mask
    lower_blue = np.array([0,0,0])
    upper_blue = np.array([180,255,255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    cv2.imshow("Blue Masked", mask_blue)


    mask = cv2.inRange(hsv, lower_range, upper_range)
    cv2.imshow("Masked", mask)

    result = cv2.bitwise_and(frame,frame,mask=mask)
    cv2.imshow("Result",result)

    # cv2.imshow("frame", frame)

    if cv2.waitKey(1) == 27:
        break

device.release()
cv2.destroyAllWindows()
