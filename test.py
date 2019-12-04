from vision import *
import cv2

img = cv2.imread("map_test_semi_complic.png")
img= cv2.flip(img, 0)

cv2.imshow("Display window", img)
warped = map_projection(img)
scale_percent = 150  # percent of original size
resized = resize_img(warped, 1.5)
thymio = detect_thymio(resized)

if thymio:
    draw_thymio(resized, thymio)

goal_pos = detect_goal(resized)
if goal_pos:
    draw_goal(resized,goal_pos)

obstacles = detect_obstacles(resized)



draw_obstacles(resized, obstacles)
cv2.imshow("projected", resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
