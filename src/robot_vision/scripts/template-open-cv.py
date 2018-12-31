import cv2
import numpy as np 

cap = cv2.VideoCapture(0)


while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red_lower = np.array([0, 160, 255])
    red_upper = np.array([180, 255, 255])

    mask = cv2.inRange(hsv, red_lower, red_upper)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()