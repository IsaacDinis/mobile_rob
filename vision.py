print("888888888")
import sys
print(sys.executable)


import cv2
import numpy as np

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
#%%

def red_detection(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([-10, 50, 50])
    upper_red = np.array([10, 255, 255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res


def edge_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    res = cv2.Canny(gray, 75, 200)
    return res


while (1):
    _, frame = cap.read()
    red = red_detection(frame)
    edge = edge_detection(frame)
    cv2.imshow('frame', frame)
    cv2.imshow('res', red)
    cv2.imshow('edge', edge)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
