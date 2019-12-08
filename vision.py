import cv2
import numpy as np
import math

colors = ["blue", "red", "pink", "green"]


def capture_image_from_webcam(webcam_number):
    cap = cv2.VideoCapture(webcam_number, cv2.CAP_DSHOW)
    while True:
        _, frame_raw = cap.read()
        display_frame = cv2.putText(frame_raw, "press space to capture", (15, 15), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imshow("webcam frame", display_frame)
        frame = cv2.transpose(frame_raw)
        frame_proj = map_projection(frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 32 and frame_proj is not None:
            frame_proj = resize_img(frame_proj, 1.5)
            cap.release()
            cv2.destroyAllWindows()
            break

        if frame_proj is not None:
            frame_proj = resize_img(frame_proj, 1.5)
            vision_img = frame_proj.copy()
            thymio_param = detect_thymio(vision_img)
            if thymio_param is not None:
                draw_thymio(vision_img, thymio_param)
            goal_pos = detect_goal(vision_img)
            if goal_pos:
                draw_goal(vision_img, goal_pos)
            obstacles = detect_obstacles(vision_img)
            draw_obstacles(vision_img, obstacles)
            cv2.imshow("vision frame", vision_img)

    return frame_proj


def resize_img(frame, scale_factor):
    width = int(frame.shape[1] * scale_factor)
    height = int(frame.shape[0] * scale_factor)
    dim = (width, height)
    resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    return resized


def color_detection(frame, color):
    with open("data\\color_calibration.txt", "r") as text_file:  # get mask values from file
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


def draw_thymio(frame, thymio_param):
    r = 100.0
    theta = thymio_param[2]
    pt1 = [thymio_param[0], thymio_param[1]]
    pt2 = [pt1[0]+r*math.cos(theta), pt1[1]+r*math.sin(theta)]
    pt1 = tuple(map(int, pt1))
    pt2 = tuple(map(int, pt2))
    cv2.arrowedLine(frame, pt1, pt2, (0, 0, 255), 3)


def draw_goal(frame, pos):
    cv2.circle(frame, pos, 1, (0, 0, 0), 3)


def calculate_orientation(pos_from, pos_to):
    v = [pos_to[0]-pos_from[0], pos_to[1]-pos_from[1]]
    v = v / np.linalg.norm(v)  # normalize the vector
    e = [1, 0]
    angle = np.sign(v[1])*np.arccos(np.clip(np.dot(v, e), -1.0, 1.0))
    return angle


def calculate_thymio_param(circles):
    thymio_param = np.zeros((1, 3))
    if circles[0, 2] < circles[1, 2]:
        thymio_param[0, 0:2] = (circles[1, 0:2])
        thymio_param[0, 2] = calculate_orientation(thymio_param[0, 0:2], circles[0, 0:2])
    else:
        thymio_param[0, 0:2] = (circles[0, 0:2])
        thymio_param[0, 2] = calculate_orientation(thymio_param[0, 0:2], circles[1, 0:2])

    return thymio_param


def detect_thymio(frame):
    _, mask = color_detection(frame, "red")
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 2:
        circles = np.empty([2, 3], dtype="float32")
        i = 0
        for cnt in contours:
            M = cv2.moments(cnt)
            try:
                circles[i, 0] = int(M["m10"] / M["m00"])
                circles[i, 1] = int(M["m01"] / M["m00"])
                circles[i, 2] = cv2.contourArea(cnt)
            except ZeroDivisionError:
                return None
            i += 1
        thymio_param = calculate_thymio_param(circles)
        return thymio_param.squeeze()

    else:
        print("thymio not found")
        return None


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
            try:
                pos.append(int(M["m10"] / M["m00"]))
                pos.append(int(M["m01"] / M["m00"]))
            except ZeroDivisionError:
                return None
            corners.append(pos)
        corners = np.asarray(corners)
        return corners

    else:
        # print("corners not found")
        return None



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
    if corners is None:
        return None
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



