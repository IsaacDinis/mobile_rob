#!/usr/bin/env python
# coding: utf-8




import os
import sys
import time
import serial
from Thymio_custom import Thymio
import numpy as np

from control import *
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
        K=2
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

def has_reached_goal(prevW, nextW, robotPos):
    proj=np.dot( robotPos-prevW, nextW-prevW)/np.linalg.norm(nextW-prevW)  #projection of robotPos onto the goal line
    return proj>np.linalg.norm(nextW-prevW)


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
    thymioTh= -np.pi/2
    robotPos=np.array([8,2])
    goal = np.array([10,10])
    start= np.array([1,1])

    waypointList= [start,goal, np.array([10,10])]
    currentGoal= 1
    intermWaypoint= None

    # epsTh = compute_eps(robotPos, goal, thymioTh )
    # turn_angle(thymio, epsTh )

    willTravelToLine=0

    navType="NavGlobal" # global variable, to modify from refresh()
    state="start"
    while 1:
        time.sleep(1) #fake particule filter Lag
        if navType=="NavGlobal":
            if has_reached_goal(start, goal, robotPos):
                print("yolo we reached goal")
                thymio.set_var("motor.right.target",0)
                thymio.set_var("motor.left.target",0)

                break
            if state=="start":
                if is_inside_tube(start, goal, robotPos, tubeTol=2):
                        epsTh = compute_eps(robotPos, goal, thymioTh)
                        if abs(epsTh)>np.deg2rad(10):
                            state = "turnInTube"
                        else:
                            state = "straightInTube"
                else:
                    state = "turnOutTube"
            print("NavType: " + navType + "state: " + state + " pos : " + str(robotPos) )

            if state=="straightInTube":
                # move_distance(thymio, np.linalg.norm(goal-start))
                thymio.set_var("motor.right.target", 80)
                thymio.set_var("motor.left.target", 80)
                state= "start"
                robotPos+=np.array([np.cos(thymioTh), np.sin(thymioTh)])*0.8 #fake odometry

            elif state== "turnInTube":
                epsTh = compute_eps(robotPos, goal, thymioTh)
                turn_angle(thymio, epsTh)
                state="wait"
                thymioTh+=epsTh #fake odom

            elif state == "wait":
                if thymio["event.args"][12] == NO_ACTION:
                    state = "start"

            elif state == "turnOutTube":
                intermWaypoint = compute_interm_waypoint(start, goal, robotPos)
                epsTh = compute_eps(robotPos, intermWaypoint, thymioTh)
                disToTravel = np.linalg.norm(robotPos - intermWaypoint)
                turn_angle_move_distance(thymio, epsTh, disToTravel)
                state = "wait"
                thymioTh += epsTh  # fake odom
                robotPos = intermWaypoint #fake odom

            else:
                print("error, state unknown")
        else: # currently switched to local nav
            state="start" # so we are in the correct state when we come back to globalNav
            print("NavType: " + navType )