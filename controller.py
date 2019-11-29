#!/usr/bin/env python
# coding: utf-8




import os
import sys
import time
import serial
from Thymio import Thymio

# sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

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

    def moveMotor(whichMotor, speed): #speed between -100 and +100
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





th = Thymio.serial(port="COM10", refreshing_rate=0.1)

time.sleep(5)

variables = th.variable_description()
for var in variables :
    print(var)





moveMotor("L", 0)
moveMotor("R", 0)
# th["prox.horizontal"]
odometryR=0
odometryL=0
Ts=0.5
lastOdom=time.time()
while odometryL<59.4:

    speedR=75
    speedL=75
    try :
        x = th.get_var("x")
        print("good 1")
    except:
        print("nope 1")
    try :
        x = th.get_variables()
        print("good 2")
    except:
        print("nope 2")
    try :
        x = th.get_var("x")
        print("good 3")
    except:
        print("nope 3")
    # my=th.list_nodes()
    # th.get_variables(57,1,my)
    distance= 70
    caliOdomL =421.9
    caliOdomR= 421.9
    th.get_variables()
    th.get_T
    ratioR=distance/caliOdomR
    ratioL=distance/caliOdomL
    # ratioR=1.0
    # ratioL=1.0
    elapsedTime = time.time() - lastOdom
    odometryR += speedR*elapsedTime*ratioR
    odometryL += speedL*elapsedTime*ratioL
    lastOdom = time.time()
    print("                                           " +str(speedL) + "     " + str(speedR))
    print(str (odometryL) + "     " + str(odometryR))
    time.sleep(0.1)

moveMotor("L", 0)
moveMotor("R", 0)
time.sleep(3)
variables = th.variable_description()

for var in variables :
    print(var)

moveMotor("L", 100)
moveMotor("R", 100)
# th["prox.horizontal"]
odometryR=0
odometryL=0
Ts=0.5
lastOdom=time.time()
while odometryL<59.4:

    speedR=100
    speedL=100
    distance= 70
    caliOdomL =421.9
    caliOdomR= 421.9

    ratioR=distance/caliOdomR
    ratioL=distance/caliOdomL
    # ratioR=1.0
    # ratioL=1.0
    elapsedTime = time.time() - lastOdom
    odometryR += speedR*elapsedTime*ratioR
    odometryL += speedL*elapsedTime*ratioL
    lastOdom = time.time()
    print("                                           " +str(speedL) + "     " + str(speedR))
    print(str (odometryL) + "     " + str(odometryR))
    time.sleep(0.1)
moveMotor("L", 0)
moveMotor("R", 0)