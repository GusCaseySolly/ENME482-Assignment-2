# -*- coding: utf-8 -*-
"""
Created on Mon Sep 22 12:09:02 2025

@author: fsmcn
"""

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm
import robodk
import tools

RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
#RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)


#DEFINE MATRICES HERE
# define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
J_intermediatepoint_1 = [-18.814643, -106.263000, -127.579643, -125.504571, -33.759643, 139.000000]
J_intermediatepoint_2 = [-7.332113, -106.007468, -126.225228, -123.837868, -36.251128, 136.406864]
#J_intermediatepoint_3 = [143.035341, -76.864448, 132.479957, -58.497983, 112.188127, 139.269728]



#Define the position where the rancilio tool sits on the mazzer scale 

offsetleft= 4
offsetforward = 0
offsetvert = -4
   
UR_T_MZS = np.array([[ 0.5,     0.86666,    0,   440.7 ],
                   [     -0.8666,      0.5,    0,  -275.3 ],
                 [              0,               0,          1,   42.4 ],
                  [              0,               0,    0,         1.000000 ]])

MZS_T_BBA = np.array([[             0,            0,    1,   -10.6 +offsetforward ],
                 [               0,              -1 ,    0,  -21.4 +offsetleft],
                 [              1,               0,     0,   14.2 +offsetvert ],
                 [              0,               0,    0,         1.000000 ]])



RCT_T_BBI = np.array([[             1,            0,    0,   -32 ],
                 [               0,              1 ,    0,  0 ],
                 [              0,               0,     1,   28.07 ],
                 [              0,               0,    0,         1.000000 ]])

TCP_T_RCT = np.array([[ np.cos(-50 * np.pi/180),     -np.sin(-50 * np.pi/180),    0, 0 ],
                 [ np.sin(-50 * np.pi/180),     np.cos(-50 * np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])
    


UR_T_TCP = UR_T_MZS @ MZS_T_BBA @ np.linalg.inv(RCT_T_BBI) @ np.linalg.inv(TCP_T_RCT )


                        
# convert numpy array into an RDK matrix
T0 = rm.Mat(UR_T_TCP.tolist())
T = robodk.UR_2_Pose(robodk.Pose_2_UR(T0))


# reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# pick up rancilio tool
tls.rancilio_tool_attach_r_ati()

# joint movement using a joint array

UR5.MoveJ(J_intermediatepoint_1, blocking=True)
UR5.MoveJ(J_intermediatepoint_2, blocking=True)

time.sleep(1)
#UR5.MoveJ(J_intermediatepoint_3, blocking=True)

# linear movement using an HT matrix
UR5.MoveL(T, blocking=True)

time.sleep(1)
tls.student_tool_detach()

time.sleep(1)
UR5.MoveJ(J_intermediatepoint_2, blocking=True)
UR5.MoveJ(J_intermediatepoint_1, blocking=True)
# go back home
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
