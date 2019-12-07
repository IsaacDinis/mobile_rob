from control import *
from Thymio_custom import Thymio
import numpy as np
from time import sleep


def check_obstacle(thymio):
    threshold = 2000
    try:
        thymio["prox.horizontal"]
    except KeyError:
        return "none"
    try:
        thymio["prox.horizontal"][4]
    except IndexError:
        return "none"

    if thymio["prox.horizontal"][0] > threshold:
        return "right"
    elif thymio["prox.horizontal"][4] > threshold:
        return "left"
    else:
        return "none"


def local_avoidance(thymio):
    state = thymio.local_nav_state
    direction = thymio.local_nav_dir
    if thymio["event.args"][12]:
        return

    if state[0] == 0:
        new_val = thymio["prox.horizontal"][2]
        old_val = 0
        while new_val >= old_val:
            if direction == "left":
                turn_angle(thymio, 0.1)  # turn left with angle = 1 rad
            elif direction == "right":
                turn_angle(thymio, -0.1)  # turn left with angle = 1 rad
            while thymio["event.args"][12]:
                pass
            old_val = new_val
            new_val = thymio["prox.horizontal"][2]
        # thymio is now facing the obstacle
        thymio.local_nav_state = [1,0]
        return

    elif state[0] == 1:
        if direction == "left":
            turn_angle(thymio, -np.pi / 2)
        elif direction == "right":
            turn_angle(thymio, np.pi / 2)
        # thymio is perpendicular to the obstacle
        thymio.local_nav_state=[2,0]
        return


    elif state[0] == 2:
        if state[1]==0:
            move_distance(thymio, 10)
            thymio.local_nav_state = [2,1]
            return
        elif state[1] == 1:
            if direction == "left":
                turn_angle(thymio, np.pi/2)
            elif direction == "right":
                turn_angle(thymio, -np.pi/2)
            thymio.local_nav_state = [2, 2]
            return
        elif state[1]==2:
            if thymio["prox.horizontal"][2] == 0:
                thymio.local_nav_state = [0,0]
                return
            elif direction == "left":
                turn_angle(thymio, -np.pi/2)
            elif direction == "right":
                turn_angle(thymio, np.pi/2)
            thymio.local_nav_state = [2,0]
            return
    return


if __name__ == "__main__":
    thymio = Thymio.serial(port="COM6", refreshing_rate=0.1)
    sleep(5)
    while True:
        local_avoidance(thymio)
