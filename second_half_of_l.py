# -*- coding: utf-8 -*-
"""
Created on Thu Oct  9 18:09:30 2025

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

J_intermediatepoint_1 = [-93.070000, -82.970000, -127.700000, -163.770000, -38.240000, -201.920000]




#Define the position where the cup tool moves


   
theta = -45

UR_T_R = np.array([[ np.cos(60* np.pi/180),     -np.sin(60* np.pi/180),     0,   -413],
                   [    np.sin(60* np.pi/180),     np.cos(60* np.pi/180),    0,  -479.4 ],
                 [              0,     0,          1,   349.3 ],
                  [              0,     0,    0,         1.000000 ]])

  

#------

R_T_RTH_pre = np.array([[             0,            0,    -1,   -14.5  ],
                 [               0,             1,    0,  69.6 ],
                 [             1,               0,    0,   -143.8 ],
                 [              0,               0,    0,         1.000000 ]])

fix = -2.4


tilt = np.array([[ np.cos(fix* np.pi/180),     0,     -np.sin(fix* np.pi/180),  0],
                [    0,              1 ,   0 ,                                              0 ],
                 [  np.sin(fix* np.pi/180),   0 ,    np.cos(fix* np.pi/180),   0 ],
                  [     0,     0,    0,         1.000000 ]])    

R_T_RTH = R_T_RTH_pre @ tilt

#-----------



RCT_T_RTH = np.array([[ 1,     0,    0, 30 ],
                 [ 0,     1,    0,  0 ],
                 [  0,    0,  1, 147.8     ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

TCP_T_RCT = np.array([[ np.cos(-50 * np.pi/180),     -np.sin(-50 * np.pi/180),    0, 0 ],
                 [ np.sin(-50 * np.pi/180),     np.cos(-50 * np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])
    
    


UR_T_TCP = UR_T_R  @ R_T_RTH  @ np.linalg.inv(RCT_T_RTH) @ np.linalg.inv(TCP_T_RCT )

T0 = rm.Mat(UR_T_TCP.tolist())
T = robodk.UR_2_Pose(robodk.Pose_2_UR(T0))
#------------------------------------------------------------------------------------------------------------------
T_rotated = np.array([[ 1,     0,     0,  0],
                   [    0,     np.cos(theta* np.pi/180),    -np.sin(theta* np.pi/180),  0 ],
                 [      0,     np.sin(theta* np.pi/180),        np.cos(theta* np.pi/180),   0 ],
                  [     0,     0,    0,         1.000000 ]])               

UR_T_TCP_rotated = UR_T_R  @ R_T_RTH @ T_rotated  @ np.linalg.inv(RCT_T_RTH) @ np.linalg.inv(TCP_T_RCT ) 

T0_rot = rm.Mat(UR_T_TCP_rotated.tolist())
T_rot = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_rot))
#----------------------------------------------------------------------------------------------------------------

T_rotated_and_down = np.array([[ 1,     0,     0,  0 -40],
                   [    0,     np.cos(theta* np.pi/180),    -np.sin(theta* np.pi/180),  0 ],
                 [      0,     np.sin(theta* np.pi/180),        np.cos(theta* np.pi/180),   0 ],
                  [     0,     0,    0,         1.000000 ]])               

UR_T_TCP_rotated_d = UR_T_R  @ R_T_RTH @ T_rotated_and_down  @ np.linalg.inv(RCT_T_RTH) @ np.linalg.inv(TCP_T_RCT ) 

T0_rot_d = rm.Mat(UR_T_TCP_rotated_d.tolist())
T_rot_d = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_rot_d))
#-------------------------------------------------------------------------------------------------------------
T_rotated_safely = np.array([[ 1,     0,     0,  0 ],
                   [    0,     np.cos(-15* np.pi/180),    -np.sin(-15* np.pi/180),  0 ],
                 [      0,     np.sin(-15* np.pi/180),        np.cos(-15* np.pi/180),   0 ],
                  [     0,     0,    0,         1.000000 ]])               

UR_T_TCP_rotated_s = UR_T_R  @ R_T_RTH @ T_rotated_safely  @ np.linalg.inv(RCT_T_RTH) @ np.linalg.inv(TCP_T_RCT ) 

T0_rot_s = rm.Mat(UR_T_TCP_rotated_s.tolist())
T_rot_s = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_rot_s))
#-------------------------------------------------------------------------------------------------------------

T_rotated_int = np.array([[ 1,     0,     0,  0 ],
                   [    0,     np.cos(-25* np.pi/180),    -np.sin(-25* np.pi/180),  0 ],
                 [      0,     np.sin(-25* np.pi/180),        np.cos(-25* np.pi/180),   0 ],
                  [     0,     0,    0,         1.000000 ]])               

UR_T_TCP_rotated_int = UR_T_R  @ R_T_RTH @ T_rotated_int  @ np.linalg.inv(RCT_T_RTH) @ np.linalg.inv(TCP_T_RCT ) 

T0_rot_int = rm.Mat(UR_T_TCP_rotated_int.tolist())
T_rot_int = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_rot_int))
#-------------------------------------------------------------------------------------------------------------

T_rotated_liner_off = np.array([[ 1,     0,     0,  0 ],
                   [    0,     np.cos(-15* np.pi/180),    -np.sin(-15* np.pi/180),  +5 ],
                 [      0,     np.sin(-15* np.pi/180),        np.cos(-15* np.pi/180),   -50 ],
                  [     0,     0,    0,         1.000000 ]])               

UR_T_TCP_rotated_o = UR_T_R  @ R_T_RTH @ T_rotated_liner_off  @ np.linalg.inv(RCT_T_RTH) @ np.linalg.inv(TCP_T_RCT ) 

T0_rot_o = rm.Mat(UR_T_TCP_rotated_o.tolist())
T_rot_o = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_rot_o))
#-------------------------------------------------------------------------------------------------------------



#T_rot_o = [-93.070000, -82.970000, -127.700000, -163.770000, -38.240000, -201.920000]



robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# pick up rancilio tool
tls.rancilio_tool_attach_r_ati()

UR5.MoveJ(J_intermediatepoint_1, blocking=True)

UR5.MoveJ(T_rot_d, blocking=True)
UR5.MoveL(T_rot, blocking=True)
UR5.MoveL(T_rot_int, blocking=True)
UR5.MoveL(T_rot_s, blocking=True)

time.sleep(1)
tls.student_tool_detach()
time.sleep(1)
UR5.MoveL(T_rot_o, blocking=True)
# go back home
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
