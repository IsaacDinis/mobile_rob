import cv2
import numpy as np
import enum
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

colors = ["blue", "red", "yellow", "green"]


def color_detection(frame, color):

    with open("color_calibration.txt", "r") as text_file:  # get mask values from file
        lines = text_file.read().splitlines()
        line = lines[colors.index(color)]
        line = list(map(int, line.split(' ')))  # converts str to int
        lower_red = np.array(line[0:3])
        upper_red = np.array(line[3:6])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.medianBlur(mask, 15)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    #res = cv2.medianBlur(res, 15)

    return res, mask


def detect_obstacles(frame):
    _, mask = color_detection(frame, 'yellow')
    apr_contours = list()
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        cnt = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        apr_contours.append(cnt)
    return apr_contours


def edge_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    res = cv2.Canny(gray, 75, 200)
    return res


def corner_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    corners = cv2.goodFeaturesToTrack(gray, 20, 0.01, 20, useHarrisDetector=True)
    if corners is not None:
        corners = np.int0(corners)
    return corners


def get_map_corners(corners):
    rect = np.zeros((4, 2), dtype="float32")
    corners = corners.reshape([corners.shape[0], 2])  # 3d matrix to 2d

    s = corners.sum(axis=1)
    rect[0] = corners[s.argmin()]
    rect[2] = corners[s.argmax()]

    diff = np.diff(corners, axis=1)
    rect[1] = corners[diff.argmin()]
    rect[3] = corners[diff.argmax()]

    return rect


def map_projection(frame,rect):
    (tl, tr, br, bl) = rect
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    # compute the perspective transform matrix and then apply it
    m = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(frame, m, (maxWidth, maxHeight))

    # return the warped image
    return warped


while 1:
    _, frame = cap.read()
    #  red = color_detection(frame, 'red')
    #blue, _ = color_detection(frame, 'blue')
    contours = detect_obstacles(frame)
    '''
    edge = edge_detection(frame)
    corners = corner_detection(frame)
    rect = get_map_corners(corners)
    '''
    '''
    if corners is not None:
           for corner in corners:
          x, y = rect.ravel()
         cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
    '''
    '''
    if corners is not None:
        for i in range(len(rect)):
            rect_int = np.int0(rect)
            cv2.circle(frame, (rect_int[i, 0], rect_int[i, 1]), 5, (0, 0, 255), -1)
    '''
    for contour in contours:
        cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)

    cv2.imshow('frame', frame)
    # cv2.imshow('red', red)
    # cv2.imshow('blue', blue)

    '''
    cv2.imshow('edge', edge)
    if rect.shape[0]==4:
        warped = map_projection(frame, rect)
        cv2.imshow('warped', warped)
    '''
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
exit()
