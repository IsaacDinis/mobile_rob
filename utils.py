import numpy as np
from numba import jit


# Utils ------------------------------------
# @jit
def norm(x, u, s):
    factor = 1./ (s * np.sqrt(2*np.pi))
    dxus = (x - u) / s
    return factor * np.exp(- dxus ** 2 / 2.)


@jit
def normalize_angle(alpha):
    while alpha > np.pi:
        alpha -= 2. * np.pi
    while alpha < -np.pi:
        alpha += 2. * np.pi
    return alpha


@jit
def normalize_angle_0_2pi(alpha):
    while alpha >= 2. * np.pi:
        alpha -= 2. * np.pi
    while alpha < 0:
        alpha += 2. * np.pi
    return alpha

@jit
def rot_mat2(angle):
    """ Create a 2D rotation matrix for angle """
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])


@jit
def xyC2W(pos):
    """ Transform an x or y coordinate in cell coordinates into world coordinates """
    return pos + 0.5

@jit
def xyW2C(pos):
    """ Transform an x or y coordinate in world coordinates into cell coordinates """
    return int(np.floor(pos))

@jit
def dxyC2W(dpos):
    """ Transform an x or y difference in cell coordinates into a difference in world coordinates """
    return float(dpos)

@jit
def dxyW2C(dpos):
    """ Transform an x or y difference in world coordinates into a difference in cell coordinates """
    return int(round(dpos))