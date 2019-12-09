"""
File containing functions to move precisely the thymio
"""

import numpy as np


ANGLE_ONLY = 1
LINE_ONLY = 2
ANGLE_LINE = 3
LINE_ANGLE = 4

MAX_DIST = 42


def turn_angle(thymio, angle):
    """ Turn the Thymio for an angle between +- 2pi, using the internal timer of Thymio"""
    thym_angle = int(angle*(2**15-1)/(2*np.pi))
    if thym_angle < 0:
        thym_angle += 2**16
    thymio.set_var("angle_set", thym_angle)
    thymio.set_var("movement_mode", ANGLE_ONLY)


def move_distance(thymio, dist):
    """ Move the Thymio for a distance between +-42cm, using the internal timer of Thymio"""
    thym_dist = int(dist*(2**15-1)/MAX_DIST)
    if thym_dist < 0:
        thym_dist += 2**16
    thymio.set_var("dist_set", thym_dist)
    thymio.set_var("movement_mode", LINE_ONLY)


def turn_angle_move_distance(thymio, angle, dist, angle_first=True):
    """Combines turn angle and move_distance, Thymio switches internally from one type of movement to the other.
    can revert to move and then turn using angle_first"""
    thym_angle = int(angle*(2**15-1)/(2*np.pi))
    if thym_angle < 0:
        thym_angle += 2**16
    thym_dist = int(dist*(2**15-1)/MAX_DIST)
    if thym_dist < 0:
        thym_dist += 2**16
    thymio.set_var("angle_set", thym_angle)
    thymio.set_var("dist_set", thym_dist)
    if angle_first:
        mode = ANGLE_LINE
    else:
        mode = LINE_ANGLE
    thymio.set_var("movement_mode", mode)


def stop_thymio(thymio):
    """ Stops the thymio right away (also clears the movement_mode flag) """
    thymio.set_var("movement_mode", 0)
    thymio.set_var("motor.left.target", 0)
    thymio.set_var("motor.right.target", 0)
