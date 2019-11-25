import cv2
import numpy as np

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)


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


def corner_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    corners = cv2.goodFeaturesToTrack(gray, 20, 0.05, 20)
    if corners is not None:
        corners = np.int0(corners)
    return corners


def get_map_corners(corners):
    rect = np.zeros((4, 2))
    corners = corners.reshape([corners.shape[0], 2])

    s = corners.sum(axis=1)
    rect[0] = corners[s.argmin()]
    rect[2] = corners[s.argmax()]

    diff = np.diff(corners, axis=1)
    rect[1] = corners[diff.argmin()]
    rect[3] = corners[diff.argmax()]

    rect = np.int0(rect)
    return rect


while 1:
    _, frame = cap.read()
    red = red_detection(frame)
    edge = edge_detection(frame)
    corners = corner_detection(frame)
    rect = get_map_corners(corners)
    #if corners is not None:
    #   for corner in corners:
     #       x, y = rect.ravel()
      #      cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    if corners is not None:
        for i in range(len(rect)):
            cv2.circle(frame, (rect[i, 0], rect[i, 1]), 5, (0, 0, 255), -1)

    cv2.imshow('frame', frame)
    cv2.imshow('res', red)
    cv2.imshow('edge', edge)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
exit()
