# -*- coding: utf-8 -*-
"""
Created on Wed Oct  8 16:12:51 2025

@author: fsmcn
"""


import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm

import tools
import robodk
import math





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


J_intermediatepoint_1 = [-117.600000, -93.070000, -114.710000, -62.770000, 89.990000, 0.000000]
J_intermediatepoint_2 = [-124.480000, -100.280000, -119.040000, -39.680000, 91.620000, 0.000000]
J_intermediatepoint_3 = [-129.140000, -71.420000, -113.270000, -90.000000, 89.990000, 0.000000]

J_intermediatepoint_x =[-123.370000, -107.490000, -110.380000, -49.770000, 89.780000, 30.460000]

fully_down = [-123.370000, -110.000000, -109.430000, -49.780000, 89.780000, 30.460000]

#Define the position where the mazzer tool moves to to unlock the mazzer scale


   
UR_T_RS = np.array([[ np.cos(-60* np.pi/180),     -np.sin(-60* np.pi/180),    0, -385.6 ],
                 [ np.sin(-60* np.pi/180),     np.cos(-60* np.pi/180),    0,  -330.5 ],
                 [              0,               0,          1,                42.5 ],
                  [              0,               0,    0,         1.000000 ]])

RS_T_SW0 = np.array([[             1,            0,    0,   31.5  +10]
,
                 [               0,              -1 ,    0,  21.55 -55],
                 [              0,               0,     -1,   -15 ],
                 [              0,               0,    0,         1.000000 ]])

RS_T_SW1 = np.array([[             1,            0,    0,   31.5 ],
                 [               0,              -1 ,    0,  21.55 ],
                 [              0,               0,     -1,   -15 ],
                 [              0,               0,    0,         1.000000 ]])



MT_T_SW = np.array([[ 1,     0,    0, 0 ],
                 [ 0,     1,    0,  0 ],
                 [  0,    0,  1, 106     ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

TCP_T_MT =  np.array([[ np.cos(-50* np.pi/180),     -np.sin(-50* np.pi/180),    0, 0 ],
                 [ np.sin(-50* np.pi/180),     np.cos(-50* np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])
    


UR_T_TCP_lock_off = UR_T_RS @ RS_T_SW0 @ np.linalg.inv(MT_T_SW) @ np.linalg.inv(TCP_T_MT )

UR_T_TCP_lock_on = UR_T_RS @ RS_T_SW1  @ np.linalg.inv(MT_T_SW) @ np.linalg.inv(TCP_T_MT )

                        
# convert numpy array into an RDK matrix
T_0B = rm.Mat(UR_T_TCP_lock_off.tolist())
T_1B = rm.Mat(UR_T_TCP_lock_on.tolist())
T_0 = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0B))
T_1 = robodk.UR_2_Pose(robodk.Pose_2_UR(T_1B))

# reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# pick up rancilio tool
tls.mazzer_tool_attach_r_ati()

# joint movement using a joint array
UR5.MoveJ(J_intermediatepoint_3, blocking=True)

#UR5.MoveJ(J_intermediatepoint_1, blocking=True)

UR5.MoveJ(J_intermediatepoint_2, blocking=True)

time.sleep(1)
# linear movement using an HT matrix
UR5.MoveL(T_1, blocking=True)


UR5.MoveJ(J_intermediatepoint_x, blocking=True)
UR5.MoveJ(fully_down, blocking=True)
time.sleep(1)
# go back home
UR5.MoveJ(J_intermediatepoint_3, blocking=True)
tls.mazzer_tool_detach_r_ati()