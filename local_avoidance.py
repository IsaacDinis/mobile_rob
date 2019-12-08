from control import *
# from Thymio_custom import Thymio
import numpy as np
from time import sleep
# import main


def check_obstacle(thymio):
    # global main.glob_ctrl

    threshold = 2200
    try:
        thymio["prox.horizontal"]
    except KeyError:
        thymio.local_nav_dir = "none"
        return
    try:
        thymio["prox.horizontal"][4]
    except IndexError:
        thymio.local_nav_dir = "none"
        return

    if thymio["prox.horizontal"][0] > threshold or thymio["prox.horizontal"][1] > threshold:
        thymio.local_nav_dir = "obs_left"
    elif thymio["prox.horizontal"][4] > threshold or thymio["prox.horizontal"][3] > threshold:
        thymio.local_nav_dir = "obs_right"
    elif thymio["prox.horizontal"][2] > 1.5 * threshold:  # last chance, we don't want this to happen
        turn_angle(thymio, -np.pi/4.)  # choose a side to go around the obstacle --> totally arbitrary
        print("Obstacle seen only from the center sensor!")
    else:
        thymio.local_nav_dir = "none"


def local_avoidance(thymio):
    state = thymio.local_nav_state
    direction = thymio.local_nav_dir
    if thymio["event.args"][12]:
        return

    if state[0] == 0:  # has to face the obstacle
        new_val = thymio["prox.horizontal"][2]
        old_val = 0
        while new_val >= old_val:
            if direction == "obs_left":
                turn_angle(thymio, 0.1)  # turn left with angle = 1 rad
            elif direction == "obs_right":
                turn_angle(thymio, -0.1)  # turn left with angle = 1 rad
            while thymio["event.args"][12]:
                pass
            old_val = new_val
            new_val = thymio["prox.horizontal"][2]
        # thymio is now facing the obstacle
        thymio.local_nav_state = [1,0]
        # print("state 1 done")
        return

    elif state[0] == 1:
        if direction == "obs_left":
            turn_angle(thymio, -np.pi / 2)
        elif direction == "obs_right":
            turn_angle(thymio, np.pi / 2)
        # thymio is perpendicular to the obstacle
        # print("state 2 done")
        thymio.local_nav_state = [2, 0]
        return

    elif state[0] == 2:
        if state[1] == 0:
            move_distance(thymio, 7)  # go parallel to the obstacle
            thymio.local_nav_state = [2, 1]
            # print("state 2.0 done")
            return
        elif state[1] == 1:
            if direction == "obs_left":
                turn_angle(thymio, np.pi/2)
            elif direction == "obs_right":
                turn_angle(thymio, -np.pi/2)
            # print("state 2.1 done")
            thymio.local_nav_state = [2, 2]
            return
        elif state[1] == 2:
            prox = thymio["prox.horizontal"][0:5]
            if max(prox) == 0:  # something detected by the Thymio?
                thymio.local_nav_state = [3, 0]
                move_distance(thymio, 15)  # move past the obstacle
                return
            elif direction == "obs_left":
                turn_angle(thymio, -np.pi/2)
            elif direction == "obs_right":
                turn_angle(thymio, np.pi/2)
            thymio.local_nav_state = [2, 0]
            # print("state 2.2  done")
            return
    elif state[0] == 3:
        thymio.local_nav_state = [0, 0]
        print("Local avoidance done!")
        thymio.nav_flag = "global"
    return


if __name__ == "__main__":
    thymio = Thymio.serial(port="COM6", refreshing_rate=0.1)
    sleep(5)
    while True:
        local_avoidance(thymio)
