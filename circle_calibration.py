import cv2
import numpy as np


def nothing(x):
    pass


use_image = True

colors = ["blue", "red", "pink", "green", "pink+red"]

if use_image:
    frame = cv2.imread("map_test.PNG")
else:
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)


def color_detection(frame, color):

    with open("color_calibration.txt", "r") as text_file:  # get mask values from file
        lines = text_file.read().splitlines()
        line = lines[colors.index(color)]
        line = list(map(int, line.split(' ')))  # converts str to int
        lower_color = np.array(line[0:3])
        upper_color = np.array(line[3:6])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.medianBlur(mask, 15)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    return res, mask


cv2.namedWindow("Trackbars")
cv2.createTrackbar("min_dist", "Trackbars", 10, 500, nothing)
cv2.createTrackbar("param1", "Trackbars", 30, 100, nothing)
cv2.createTrackbar("param2", "Trackbars", 40, 100, nothing)
cv2.createTrackbar("minRadius", "Trackbars", 0, 100, nothing)
cv2.createTrackbar("maxRadius", "Trackbars", 0, 300, nothing)


print("press space when done ")

while True:
    if use_image:
        frame = cv2.imread("map_test.PNG")
    else:
        _, frame = cap.read()

    image = cv2.putText(frame, "press space when done ", (15, 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)

    min_dist = cv2.getTrackbarPos("min_dist", "Trackbars")
    param1 = cv2.getTrackbarPos("param1", "Trackbars")
    param2 = cv2.getTrackbarPos("param2", "Trackbars")
    minRadius = cv2.getTrackbarPos("minRadius", "Trackbars")
    maxRadius = cv2.getTrackbarPos("maxRadius", "Trackbars")

    _, mask = color_detection(frame, "pink+red")
    #circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.5, min_dist, param1, param2, minRadius, maxRadius)
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, min_dist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

    if circles is not None:
        detected_circles = np.uint16(np.around(circles))
        for (x, y, r) in detected_circles[0, :]:
            cv2.circle(frame, (x, y), r, (0, 0, 0), 3)
            cv2.circle(frame, (x, y), 2, (0, 255, 255), 3)

    cv2.imshow("frame", frame)

    key = cv2.waitKey(10)

    if key == 32:  # space pressed
        with open("circle_calibration.txt", "w+") as text_file:
            print(f"{min_dist} {param1} {param2} {minRadius} {maxRadius}", file=text_file)
        break

    if key == 27:  # esc pressed
        break
if not use_image:
    cap.release()
cv2.destroyAllWindows()
