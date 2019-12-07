from control import *
from Thymio_custom import Thymio
import numpy as np
from time import sleep


def check_obstacle(self):
    threshold = 2000
    if thymio["prox.horizontal"][0] > threshold:
        return "right"
    elif thymio["prox.horizontal"][4] > threshold:
        return "left"
    else:
        return "none"


def local_avoidance(thymio, direction):

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
    if direction == "left":
        turn_angle(thymio, -np.pi / 2)
    elif direction == "right":
        turn_angle(thymio, np.pi / 2)
    while thymio["event.args"][12]:
        pass

    while True:
        move_distance(thymio, 10)
        while thymio["event.args"][12]:
            pass
        if direction == "left":
            turn_angle(thymio, np.pi/2)
        elif direction == "right":
            turn_angle(thymio, -np.pi/2)
        while thymio["event.args"][12]:
            pass
        if thymio["prox.horizontal"][2] == 0:
            break
        if direction == "left":
            turn_angle(thymio, -np.pi/2)
        elif direction == "right":
            turn_angle(thymio, np.pi/2)
        while thymio["event.args"][12]:
            pass

    return


if __name__ == "__main__":
    thymio = Thymio.serial(port="COM6", refreshing_rate=0.1)
    sleep(5)
    while True:
        local_avoidance(thymio)
