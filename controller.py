#!/usr/bin/env python
# coding: utf-8




import os
import sys
import time
import serial
from Thymio_custom import Thymio
import numpy as np

from control import turn_angle, move_distance
from utils import normalize_angle_0_2pi, normalize_angle_minus_pi_plus_pi

NO_ACTION = 0
MOV_ANGLE = 1
MOV_STRAIGHT = 2

def remap(x, in_min, in_max, out_min, out_max, constrain=True):
    out=float(x - in_min) * float(out_max - out_min) / float(in_max - in_min) + out_min
    if constrain:
        if out_max>out_min:
            if out>out_max:
                out=out_max
            if out<out_min:
                out=out_min
        else:
            if out<out_max:
                out=out_max
            if out>out_min:
                out=out_min
    return float(x - in_min) * float(out_max - out_min) / float(in_max - in_min) + out_min

def moveMotor(whichMotor, speed, th): #speed between -100 and +100
    newSpeed= remap(speed, -100, 100, -512, 512)
    if whichMotor=="L":
        motorName="motor.left.target"
    elif whichMotor=="R":
        motorName = "motor.right.target"
    else:
        motorName="problem"
    if newSpeed >=0:
        th.set_var(motorName, int(newSpeed))
    else:
        newSpeed=int(abs(newSpeed))
        th.set_var(motorName, 2 ** 16 - newSpeed)


def compute_eps( thymioPos, goalPos, thymioTh):
    thG = np.arctan2(goalPos[1]-thymioPos[1], goalPos[0]-thymioPos[0] )
    thG=  normalize_angle_0_2pi(thG)
    thymioTh=normalize_angle_0_2pi(thymioTh)

    epsTh = thG - thymioTh

    epsTh=normalize_angle_minus_pi_plus_pi(epsTh)
    return epsTh

def compute_interm_waypoint(prevW, nextW, robotPos):
    """k= added distance from projection, here K = distance from robot to preojection, so the robot will come back to line with 45degres, \n
     smaller K = quicker sharper return on the line. \n
     K = 0 --> direct return on the line at 90 degres"""

    while 1:
        K=1
        intermW = prevW + (nextW - prevW) / np.linalg.norm(nextW - prevW) * (
                np.dot(robotPos - prevW, nextW - prevW) / np.linalg.norm(nextW - prevW) + K)

        if np.linalg.norm(intermW - prevW) > np.linalg.norm(nextW - prevW):   # if imtermediate point is further than next point
            K=K/2 #go back faster to the line
        else:
            break
    return intermW


def is_inside_tube(prevW, nextW, robotPos, tubeTol):
    """ tubeTol = half the tube total diameter"""

    M= np.dot(nextW-prevW, robotPos-prevW)/np.linalg.norm(nextW-prevW)**2 * (nextW-prevW)   +prevW
    if np.linalg.norm(M-robotPos)>tubeTol:
        return False
    else:
        return True

if __name__ == "__main__":
# %%
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
        time.sleep(0.5)


    # thymioPos = [1,1]
    # goalPos = [1,0]
    thymioTh= np.pi/2
    robotPos=np.array([2,2])
    goal = np.array([5,5])
    start= np.array([1,1])

    waypointList= [start,goal, np.array([10,10])]
    currentGoal= 1
    intermWaypoint= None

    # epsTh = compute_eps(robotPos, goal, thymioTh )
    # turn_angle(thymio, epsTh )

    willTravelToLine=0



navType="Global" # global variable, to modify from refresh()
state="start"
while 1:
    time.sleep(2) #fake particule filter Lag
    if state=="start":
        if is_inside_tube(start, goal, robotPos, tubeTol=2):
                epsTh = compute_eps(robotPos, goal, thymioTh)
                if abs(epsTh)>np.deg2rad(5):
                    state = "turnInTube"
                else:
                    state = "straightInTube"
        else:
            state = "turnOutTube"
    if state=="straightInTube":
        move_distance()
        gostraight( )









    time.sleep(0.5)
    #     move_distance(thymio, 10)



