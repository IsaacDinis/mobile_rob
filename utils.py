"""
Utilitary functions, optimised with numba
"""

import numpy as np
from numba import jit

@jit(nopython=True)
def norm(x, u, s):
    factor = 1. / (s * np.sqrt(2.*np.pi))
    dxus = (x - u) / s
    return factor * np.exp(- dxus ** 2. / 2.)


@jit(nopython=True)
def normalize_angle(alpha):
    while alpha > np.pi:
        alpha -= 2. * np.pi
    while alpha < -np.pi:
        alpha += 2. * np.pi
    return alpha


@jit(nopython=True)
def normalize_angle_0_2pi(alpha):
    while alpha >= 2. * np.pi:
        alpha -= 2. * np.pi
    while alpha < 0:
        alpha += 2. * np.pi
    return alpha
@jit(nopython=True)
def normalize_angle_minus_pi_plus_pi(alpha):
    while alpha >= np.pi:
        alpha -= 2. * np.pi
    while alpha <  -np.pi:
        alpha += 2. * np.pi
    return alpha

@jit(nopython=True)
def rot_mat2(angle):
    """ Create a 2D rotation matrix for angle """
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])


@jit(nopython=True)
def xyC2W(pos):
    """ Transform an x or y coordinate in cell coordinates into world coordinates """
    return pos + 0.5


@jit(nopython=True)
def xyW2C(pos):
    """ Transform an x or y coordinate in world coordinates into cell coordinates """
    return int(np.floor(pos))


@jit(nopython=True)
def dxyC2W(dpos):
    """ Transform an x or y difference in cell coordinates into a difference in world coordinates """
    return float(dpos)


@jit(nopython=True)
def dxyW2C(dpos):
    """ Transform an x or y difference in world coordinates into a difference in cell coordinates """
    return int(round(dpos))


@jit(nopython=True)
def is_in_bound_cell(shape, x, y):
    """ Return whether a given position x,y (as int) is within the bounds of a 2D array """
    if 0 <= x < shape[0] and 0 <= y < shape[1]:
        return True
    else:
        return False


@jit(nopython=True)
def is_in_bound(shape, pos):
    """ Check whether a given position is within the bounds of a 2D array """
    # assert pos.shape[0] == 2
    x = xyW2C(pos[0])
    y = xyW2C(pos[1])
    return is_in_bound_cell(shape, x, y)

def unit_to_sensor(value, table):
    assert len(table) == 17
    table_bin = int(value * 16)
    r = value - (1. / 16.) * table_bin
    if table_bin == 16:
        return table[16]
    return float(table[table_bin]) * (1. - r) + float(table[table_bin + 1]) * r
