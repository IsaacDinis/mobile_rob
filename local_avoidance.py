from control import *
from Thymio_custom import Thymio
import numpy as np
from time import sleep

def local_avoidance(thymio):
    threshold = 2000
    if thymio["prox.horizontal"][0] > threshold:
        new_val = thymio["prox.horizontal"][2]
        old_val = 0
        while new_val >= old_val:
            turn_angle(thymio, 1)  # turn left with angle = 1 rad
            old_val = new_val
            new_val = thymio["prox.horizontal"][2]
        # thymio is now facing the obstacle
        turn_angle(thymio, -np.pi / 2)

        while True:
            move_distance(thymio, 5)
            turn_angle(thymio, np.pi/2)
            if thymio["prox.horizontal"][2] == 0:
                break
            turn_angle(thymio, -np.pi / 2)
    return


if __name__ == "__main__":
    thymio = Thymio.serial(port="COM31", refreshing_rate=0.1)
    while True:
        sleep(1)
        local_avoidance(thymio)
