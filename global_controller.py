#!/usr/bin/env python
# coding: utf-8




import os
import sys
import time
import serial
# from Thymio_custom import Thymio
import numpy as np

from control import *
from utils import normalize_angle_0_2pi, normalize_angle_minus_pi_plus_pi


class GlobalController:
    """ Princial function: followPath
    """
    NO_ACTION = 0
    MOV_ANGLE = 1
    MOV_STRAIGHT = 2

    def __init__(self, path, tubeTol = 2, outOfTubeAvancementTarget = 2, angleInTubeTol = 10, cornerCut=2, noTurningDistance=6):
        self.tubeTol = tubeTol
        self.outOfTubeAvancementTarget = outOfTubeAvancementTarget
        self.angleInTubeTol = angleInTubeTol
        self.currentTargetID = 1
        self.state = "start"
        self.path = path
        self.cornerCut=cornerCut
        self.noTurningDistance=noTurningDistance

    @staticmethod
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

    @staticmethod
    def moveMotor(whichMotor, speed, th): #speed between -100 and +100
        newSpeed= GlobalController.remap(speed, -100, 100, -512, 512)
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

    @staticmethod
    def compute_eps( thymioPos, goalPos, thymioTh):
        thG = np.arctan2(goalPos[1]-thymioPos[1], goalPos[0]-thymioPos[0] )
        thG=  normalize_angle_0_2pi(thG)
        thymioTh=normalize_angle_0_2pi(thymioTh)

        epsTh = thG - thymioTh

        epsTh=normalize_angle_minus_pi_plus_pi(epsTh)
        return epsTh

    @staticmethod
    def compute_interm_waypoint(prevW, nextW, robotPos, K):
        """k= added distance from projection, here K = distance from robot to preojection, so the robot will come back to line with 45degres, \n
         smaller K = quicker sharper return on the line. \n
         K = 0 --> direct return on the line at 90 degres"""

        while 1:
            intermW = prevW + (nextW - prevW) / np.linalg.norm(nextW - prevW) * (
                    np.dot(robotPos - prevW, nextW - prevW) / np.linalg.norm(nextW - prevW) + K)

            if np.linalg.norm(intermW - prevW) > np.linalg.norm(nextW - prevW):   # if imtermediate point is further than next point
                K=K/2 #go back faster to the line
            else:
                break
        return intermW

    @staticmethod
    def is_inside_tube(prevW, nextW, robotPos, tubeTol):
        """ tubeTol = half the tube total diameter"""

        M= np.dot(nextW-prevW, robotPos-prevW)/np.linalg.norm(nextW-prevW)**2 * (nextW-prevW)   +prevW
        if np.linalg.norm(M-robotPos)>tubeTol:
            return False
        else:
            return True

    @staticmethod
    def has_reached_nextW(prevW, nextW, robotPos, cornerCut):
        proj=np.dot( robotPos-prevW, nextW-prevW)/np.linalg.norm(nextW-prevW)  #projection of robotPos onto the goal line
        return (proj+cornerCut)>np.linalg.norm(nextW-prevW)

    def followPath(self, thymioPos, thymioTh, thymio, navType):  # ATTENTION Changé l'ordre de theta et pos - Loic
        """ fct to navigate in and out a tube on a given set of waypoint (always give the total self.path including
        starting waypoint.
        self.currentTargetID shall be set to 1 in the beginning"""

        # global navType
        if navType == "global":
            nextW = self.path[self.currentTargetID]
            lastW = self.path[self.currentTargetID - 1]
            if GlobalController.has_reached_nextW(lastW, nextW, thymioPos, self.cornerCut):
                print("We reached a waypoint")
                thymio.set_var("motor.right.target", 0)
                thymio.set_var("motor.left.target", 0)
                if self.currentTargetID == len(self.path) - 1:
                    print("the waypoint we reached was the goal ! ")
                    self.state = "reachedGoal"
                else:
                    self.currentTargetID += 1
                    nextW = self.path[self.currentTargetID]
                    lastW = self.path[self.currentTargetID - 1]

            if self.state == "start":
                if GlobalController.is_inside_tube(lastW, nextW, thymioPos, self.tubeTol):
                    epsTh = GlobalController.compute_eps(thymioPos, nextW, thymioTh)
                    if abs(epsTh) > np.deg2rad(self.angleInTubeTol) :
                        if not GlobalController.has_reached_nextW(lastW, nextW, thymioPos, self.noTurningDistance):
                            self.state = "turnInTube"
                        else:
                            print("inside the NO turning distance")
                            self.state = "straightInTube"
                    else:
                        self.state = "straightInTube"
                else:
                    if not GlobalController.has_reached_nextW(lastW, nextW, thymioPos, self.noTurningDistance):
                        print("inside the NO turning distance")
                        self.state = "turnOutTube"
                    else:
                        self.state=self.state = "straightInTube"
            print("NavType: " + navType + " // self.state: " + self.state + " // pos : " + str(thymioPos))

            if self.state == "straightInTube":
                # move_distance(thymio, np.linalg.norm(nextW-lastW))
                thymio.set_var("motor.right.target", 80)
                thymio.set_var("motor.left.target", 80)
                self.state = "start"
                # thymioPos += np.array([np.cos(thymioTh), np.sin(thymioTh)]) * 0.8  # fake odometry

            elif self.state == "turnInTube":
                epsTh = GlobalController.compute_eps(thymioPos, nextW, thymioTh)
                turn_angle(thymio, epsTh)
                print("turning in tube " + str(epsTh))
                self.state = "wait"
                # thymioTh += epsTh  # fake odometry

            elif self.state == "wait":
                if thymio["event.args"][12] == GlobalController.NO_ACTION:
                    self.state = "start"

            elif self.state == "turnOutTube":
                intermWaypoint = GlobalController.compute_interm_waypoint(lastW, nextW, thymioPos, self.outOfTubeAvancementTarget)
                epsTh = GlobalController.compute_eps(thymioPos, intermWaypoint, thymioTh)
                disToTravel = np.linalg.norm(thymioPos - intermWaypoint)
                turn_angle_move_distance(thymio, epsTh, disToTravel)
                self.state = "wait"
                # thymioTh += epsTh  # fake odometry
                # thymioPos = intermWaypoint  # fake odometry

            elif self.state == "reachedGoal":
                print("the waypoint we reached was the goal ! ")

            else:
                print("error, self.state unknown")
        else:  # currently switched to local nav
            self.state = "start"  # so we are in the correct self.state when we come back to globalNav
            print("NavType: " + navType)

        # fake odom
        # return thymioTh, thymioPos





if __name__ == "__main__":
# %%
    thymio = Thymio.serial(port="COM18", refreshing_rate=0.1)

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

    navType = "global"  # TODO isaac : refresh() is going to modify this var
    globalcontroller= GlobalController([np.array([1, 1]), np.array([10, 10]), np.array([15, 5])] )
    while 1:
        time.sleep(1) #fake lag for future particule filter
        thymioTh = np.pi  # TODO Loic : mettre la valeur d'angle du thymio ici
        thymioPos = np.array([2, 6])  # TODO Loic : mettre la position du thymio ici
        globalcontroller.followPath(thymioTh, thymioPos, thymio)