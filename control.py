import numpy as np


import utils as ut
from Thymio_custom import Thymio

MOV_ANGLE = 1
MOV_STRAIGHT = 2


def turn_angle(thymio, angle):
    """ Turn the Thymio for an angle between +- 2pi"""
    thym_angle = round(angle*(2**15-1)/(2*np.pi))
    if thym_angle < 0:
        thym_angle += 2**16
    thymio.set_var("angle_set", int(thym_angle) )
    thymio.set_var("movement_mode", MOV_ANGLE)


def move_distance(thymio, dist):
    """ Move the Thymio for a distance between +-20cm"""
    thym_dist = round(dist*(2**15-1)/20)
    if thym_dist < 0:
        thym_dist += 2**16
    thymio.set_var("dist_set", thym_dist)
    thymio.set_var("movement_mode", MOV_STRAIGHT)


if __name__ == "__main__":
# %%
    import time
    thymio = Thymio.serial(port="COM20", refreshing_rate=0.1)


    ok = False
    yy, zz = [], []
    while not ok or len(zz) == 0:
        time.sleep(0.5)
        try:
            thymio["event.args"] = [0]*32
            zz = thymio["prox.ground.delta"]
        except KeyError:
            time.sleep(0.1)
        else:
            ok = True
        time.sleep(5)
    print(thymio["event.args"][12])
    print("turning_now!")
    turn_angle(thymio, np.pi / 4)
    while 1:
        print(thymio["event.args"][12])
        time.sleep(0.1)

