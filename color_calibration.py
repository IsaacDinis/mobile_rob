import cv2
import numpy as np


def nothing(x):
    pass


open('color_calibration.txt', 'w+').close()  # clear file
colors = ["blue", "red", "yellow", "green"]
i = 0

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
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
    _, frame = cap.read()
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
    result = cv2.bitwise_and(frame, frame, mask=mask)

    image = cv2.putText(result, text, (15, 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("result", result)
    key = cv2.waitKey(1)

    if key == 32:  # space pressed
        i += 1
        with open("color_calibration.txt", "a") as text_file:
            print(f"{l_h} {l_s} {l_v} {u_h} {u_s} {u_v}", file=text_file)
        if i == len(colors):
            break
        else:
            text = "set {} detection, press space when done".format(colors[i])
            print(text)

    if key == 27:  # esc pressed
        break

cap.release()
cv2.destroyAllWindows()
