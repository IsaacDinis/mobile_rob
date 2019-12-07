import cv2
import numpy as np
from objects import *
import math

colors = ["blue", "red", "pink", "green"]

def capture_image_from_webcam(webcam_number):
    cap = cv2.VideoCapture(webcam_number, cv2.CAP_DSHOW)
    while True:
        _, frame = cap.read()
        image = cv2.putText(frame, "press space to capture", (15, 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Display window", frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 32:
            cap.release()
            cv2.destroyAllWindows()
            break
    frame = cv2.flip(frame, 0)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = map_projection(frame)
    frame = resize_img(frame, 1.5)
    return frame


def resize_img(frame, scale_factor):
    width = int(frame.shape[1] * scale_factor)
    height = int(frame.shape[0] * scale_factor)
    dim = (width, height)
    resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    return resized


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


def draw_obstacles(frame, obstacles):
    for obst in obstacles:
        obst = obst.squeeze()
        for i in range(len(obst)):
            pos = tuple(obst[i])
            cv2.circle(frame, pos, 1, (0, 0, i*60), 3)


def draw_thymio(frame, thymio):
    r = 100.0
    theta = thymio.theta
    pt1 = [thymio.pos.x, thymio.pos.y]
    pt2 = [pt1[0]+r*math.cos(theta), pt1[1]+r*math.sin(theta)]
    pt1 = tuple(map(int, pt1))
    pt2 = tuple(map(int, pt2))
    cv2.arrowedLine(frame, pt1, pt2, (0, 0, 0), 3)


def draw_goal(frame, pos):
    cv2.circle(frame, pos, 1, (0, 0, 0), 3)


def detect_thymio(frame):
    _, mask = color_detection(frame, "red")
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 2:
        circles = np.empty([2, 3], dtype="float32")
        i = 0
        for cnt in contours:
            M = cv2.moments(cnt)
            circles[i, 0] = int(M["m10"] / M["m00"])
            circles[i, 1] = int(M["m01"] / M["m00"])
            circles[i, 2] = cv2.contourArea(cnt)
            i += 1
            #  cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        thymio = Thymio(circles)
        return thymio

    else:
        print("thymio not found")
        return 0


def detect_goal(frame):
    _, mask = color_detection(frame, "pink")
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 1:
        pos = list()

        cnt = contours[0]
        M = cv2.moments(cnt)
        pos.append(int(M["m10"] / M["m00"]))
        pos.append(int(M["m01"] / M["m00"]))
        pos = tuple(pos)
        return pos

    else:
        print("goal not found")
        return 0


def detect_circles(frame, color):
    _, mask = color_detection(frame, color)
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.5, 10, param1=50, param2=10, minRadius=0, maxRadius=0)
    return circles


def detect_obstacles(frame):
    _, mask = color_detection(frame, "green")
    apr_contours = list()
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        cnt = cv2.approxPolyDP(cnt, 0.05*cv2.arcLength(cnt, True), True)
        # cnt = np.flipud(cnt)
        apr_contours.append(cnt)
    return apr_contours


def edge_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    res = cv2.Canny(gray, 75, 200)
    return res


def detect_corners(frame, color):
    _, mask = color_detection(frame, "blue")
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 4:
        corners = list()
        for i in range(4):
            pos = list()
            cnt = contours[i]
            M = cv2.moments(cnt)
            pos.append(int(M["m10"] / M["m00"]))
            pos.append(int(M["m01"] / M["m00"]))
            corners.append(pos)
        corners = np.asarray(corners)
        return corners

    else:
        print("corners not found")
        return 0



def get_map_corners(corners):
    rect = np.zeros((4, 2), dtype="float32")
    s = corners.sum(axis=1)
    rect[0] = corners[s.argmin()]
    rect[2] = corners[s.argmax()]

    diff = np.diff(corners, axis=1)
    rect[1] = corners[diff.argmin()]
    rect[3] = corners[diff.argmax()]

    return rect


def map_projection(frame):

    corners = detect_corners(frame, "blue")
    rect = get_map_corners(corners)

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


if __name__ == '__main__':
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    while 1:
        _, frame = cap.read()
        #  red = color_detection(frame, 'red')
        blue, _ = color_detection(frame, 'blue')
        #  contours = detect_obstacles(frame)
        # contour = detect_thymio(frame)
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
        # for contour in contours:
        # if len(contour):
        #     cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
        detect_thymio(frame)
        circles = detect_circles(frame)
        if circles is not None:
            detected_circles = np.uint16(np.around(circles))
            for (x, y, r) in detected_circles[0, :]:
                cv2.circle(frame, (x, y), r, (0, 0, 0), 3)
                cv2.circle(frame, (x, y), 2, (0, 255, 255), 3)

        cv2.imshow('frame', frame)
        # cv2.imshow('red', red)
        cv2.imshow('blue', blue)

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

