import cv2
import numpy as np
import sys


def nothing(x):
    pass


use_image = False


open('data\\color_calibration.txt', 'w+').close()  # clear file
colors = ["blue", "red", "pink", "green"]
i = 0

if use_image:
    frame = cv2.imread("map_test/map_test.PNG")
else:
    cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)  # CHANGE CAM NUMBER HERE
    while True:
        _, frame = cap.read()
        image = cv2.putText(frame, "press space to capture", (15, 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Display window", frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            sys.exit(0)
        elif k == 32:
            _, frame = cap.read()
            cap.release()
            cv2.destroyAllWindows()
            break

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

text = "set {} detection, press space when done".format(colors[0])
print(text)

while True:

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    lower_color = np.array([l_h, l_s, l_v])
    upper_color = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.medianBlur(mask, 5)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    image = cv2.putText(result, text, (15, 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("result", result)
    key = cv2.waitKey(1)

    if key == 32:  # space pressed
        i += 1
        with open("data\\color_calibration.txt", "a") as text_file:
            print(f"{l_h} {l_s} {l_v} {u_h} {u_s} {u_v}", file=text_file)
            cv2.setTrackbarPos("L - H", "Trackbars", 0)
            cv2.setTrackbarPos("L - S", "Trackbars", 0)
            cv2.setTrackbarPos("L - V", "Trackbars", 0)
            cv2.setTrackbarPos("U - H", "Trackbars", 179)
            cv2.setTrackbarPos("U - S", "Trackbars", 255)
            cv2.setTrackbarPos("U - V", "Trackbars", 255)

        if i == len(colors):
            break
        else:
            text = "set {} detection, press space when done".format(colors[i])
            print(text)

    if key == 27:  # esc pressed
        break
if not use_image:
    cap.release()
cv2.destroyAllWindows()
