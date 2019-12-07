from vision import *
import cv2

img = cv2.imread("map_test/map_test.png")
# cap = cv2.VideoCapture(1 , cv2.CAP_DSHOW)
# while True:
#     _, img = cap.read()
#     cv2.imshow("Display window", img)
#     k = cv2.waitKey(5) & 0xFF
#     if k == 27:
#         break
# img = cv2.flip(img, 0)
# img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

cv2.imshow("Display window", img)
warped = map_projection(img)
scale_percent = 150  # percent of original size
resized = resize_img(warped, 1.5)
thymio = detect_thymio(resized)

if thymio:
    draw_thymio(resized, thymio)

goal_pos = detect_goal(resized)
if goal_pos:
    draw_goal(resized, goal_pos)

obstacles = detect_obstacles(resized)



draw_obstacles(resized, obstacles)
cv2.imshow("projected", resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
# cap.release()
