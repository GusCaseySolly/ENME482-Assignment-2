# -*- coding: utf-8 -*-
"""
Created on Mon Sep 22 12:09:02 2025

@author: fsmcn
"""

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm

import tools
import robodk
import math






RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
#RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)


#DEFINE MATRICES HERE
# define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)

J_intermediatepoint_1 = [-35.350000, -85.850000, -126.250000, -58.440000, 88.740000, -5.100000]
J_intermediatepoint_2 = [-28.120000, -104.880000, -113.020000, -47.960000, 90.000000, -6.840000]


#Define the position where the mazzer tool moves to to unlock the mazzer scale

offsetleft= 0
offsetforward = -30
offsetvert = -10
   
UR_T_MZS = np.array([[ 0.5,     0.86666,    0,   440.7 ],
                   [     -0.8666,      0.5,    0,  -275.3 ],
                 [              0,               0,          1,   42.4 ],
                  [              0,               0,    0,         1.000000 ]])

MZS_T_SW0 = np.array([[             1,            0,    0,   31.5  ],
                 [               0,              -1 ,    0,  -56.38 ],
                 [              0,               0,     -1,   -15  ],
                 [              0,               0,    0,         1.000000 ]])

MZS_T_SW1 = np.array([[             1,            0,    0,   31.5 +  offsetforward ],
                 [               0,              -1 ,    0,  -56.38 +  offsetleft],
                 [              0,               0,     -1,   -15 +  offsetvert ],
                 [              0,               0,    0,         1.000000 ]])



MT_T_SW = np.array([[ 1,     0,    0, 0 ],
                 [ 0,     1,    0,  0 ],
                 [  0,    0,  1, 106     ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

TCP_T_MT =  np.array([[ np.cos(-50* np.pi/180),     -np.sin(-50* np.pi/180),    0, 0 ],
                 [ np.sin(-50* np.pi/180),     np.cos(-50* np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])
    


UR_T_TCP_switch_off = UR_T_MZS @ MZS_T_SW0 @ np.linalg.inv(MT_T_SW) @ np.linalg.inv(TCP_T_MT )

UR_T_TCP_switch_on = UR_T_MZS @ MZS_T_SW1  @ np.linalg.inv(MT_T_SW) @ np.linalg.inv(TCP_T_MT )

                        
# convert numpy array into an RDK matrix
T_0B = rm.Mat(UR_T_TCP_switch_off.tolist())
T_1B = rm.Mat(UR_T_TCP_switch_on.tolist())
T_0 = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0B))
T_1 = robodk.UR_2_Pose(robodk.Pose_2_UR(T_1B))

# reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# pick up rancilio tool
tls.mazzer_tool_attach_r_ati()

# joint movement using a joint array

UR5.MoveJ(J_intermediatepoint_1, blocking=True)

UR5.MoveJ(J_intermediatepoint_2, blocking=True)

time.sleep(1)
# linear movement using an HT matrix
UR5.MoveL(T_0, blocking=True)


UR5.MoveL(T_1, blocking=True)

time.sleep(1)
# go back home
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)