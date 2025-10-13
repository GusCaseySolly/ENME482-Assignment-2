# -*- coding: utf-8 -*-
"""
Created on Thu Sep 18 13:52:46 2025

@author: Owner
"""

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm

import tools

RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
#RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)


#DEFINE MATRICES HERE
# define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
J_intermediatepoint1 = [-39.780000, -117.350000, -109.390000, -125.300000, -69.610000, -220.770000]
J_intermediatepoint2 = [-27.850000, -89.500000, -139.230000, -125.300000, -69.610000, -220.770000]
J_intermediatepoint3 = [0.000000, -79.560000, -151.160000, -87.510000, -23.870000, 0.000000]


#Define the position where the mazzer tool slots into the cup dispensor index


UR_T_MF = np.array([[-0.87898362,  0.1756593,  -0.44331885, 502],
                    [-0.47685197, -0.32379368,  0.81717187, -419.6],
                    [ 0.,         0.92967815,  0.36837282, 318.6],
                    [0,0,0,1]])

MT_T_PB = np.array([[ 1,     0,    0, 0 ],
                 [ 0,     1,    0,  0 ],
                 [  0,    0,  1, 106    ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

TCP_T_MT =  np.array([[ np.cos(-50* np.pi/180),     -np.sin(-50* np.pi/180),    0, 0 ],
                 [ np.sin(-50* np.pi/180),     np.cos(-50* np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

#import numpy as np
pb_os_x = 10
pb_os_x_pos = 16
pb_os_z = 20
pb_os_y = 10
pb_boff_os = 20
MF_T_PB = np.array([
    [-0.87898362,  0.47685197,  0.,        91.1],
    [-0.17476683, -0.32214857,  0.93041757,-189.1],
    [ 0.44367145,  0.8178218,   0.36650123,-144.2],
    [ 0.,          0.,          0.,          1. ]
])

MF_T_PB_xos = np.array([
    [-0.87898362,  0.47685197,  0.,        91.1-pb_os_x],
    [-0.17476683, -0.32214857,  0.93041757,-189.1],
    [ 0.44367145,  0.8178218,   0.36650123,-144.2],
    [ 0.,          0.,          0.,          1. ]
])

MF_T_PB_xos_pos = np.array([
    [-0.87898362,  0.47685197,  0.,        91.1+pb_os_x_pos],
    [-0.17476683, -0.32214857,  0.93041757,-189.1],
    [ 0.44367145,  0.8178218,   0.36650123,-144.2],
    [ 0.,          0.,          0.,          1. ]
])

MF_T_PB_yos = np.array([
    [-0.87898362,  0.47685197,  0.,        91.1-pb_boff_os],
    [-0.17476683, -0.32214857,  0.93041757,-189.1-pb_os_y],
    [ 0.44367145,  0.8178218,   0.36650123,-144.2],
    [ 0.,          0.,          0.,          1. ]
])

MF_T_PB_zos = np.array([
    [-0.87898362,  0.47685197,  0.,        91.1],
    [-0.17476683, -0.32214857,  0.93041757,-189.1],
    [ 0.44367145,  0.8178218,   0.36650123,-144.2-pb_os_z],
    [ 0.,          0.,          0.,          1. ]
])

R = np.array([
    [1, 0, 0, 0],
    [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)), 0],
    [0, np.sin(np.deg2rad(90)),  np.cos(np.deg2rad(90)), 0],
    [0, 0, 0, 1]
    ])

# Overwrite with rotated version
MF_T_PB = MF_T_PB @ R
MF_T_PB_xos = MF_T_PB_xos @ R
MF_T_PB_xos_pos = MF_T_PB_xos_pos @ R
MF_T_PB_yos = MF_T_PB_yos @ R
MF_T_PB_zos = MF_T_PB_zos @ R



UR_T_TCP = UR_T_MF @ MF_T_PB @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_xos = UR_T_MF @ MF_T_PB_xos @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_xos_pos = UR_T_MF @ MF_T_PB_xos_pos @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_yos = UR_T_MF @ MF_T_PB_yos @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_zos = UR_T_MF @ MF_T_PB_zos @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
# convert numpy array into an RDK matrix
T = rm.Mat(UR_T_TCP.tolist())
T_xos = rm.Mat(UR_T_TCP_xos.tolist())
T_xos_pos = rm.Mat(UR_T_TCP_xos_pos.tolist())
T_yos = rm.Mat(UR_T_TCP_yos.tolist())
T_zos = rm.Mat(UR_T_TCP_zos.tolist())

# reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# pick up mazzer tool
tls.mazzer_tool_attach_r_ati()

# joint movement using a joint array
UR5.MoveJ(J_intermediatepoint2, blocking=True)
#UR5.MoveJ(J_intermediatepoint3, blocking=True)
UR5.MoveJ(J_intermediatepoint1, blocking=True)


time.sleep(1)

# linear movement using an HT matrix
# UR5.MoveJ(T_vos, blocking=True)

# time.sleep(1)

UR5.MoveJ(T, blocking = True)

time.sleep(1)

UR5.MoveL(T_xos,blocking=True)

time.sleep(1)

UR5.MoveL(T_xos_pos, blocking = True)

before_2nd_press = [-55.320000, -126.040000, -94.740000, -137.670000, -137.680000, -310.210000]
second_press = [-54.200000, -126.050000, -94.740000, -137.680000, -137.680000, -310.210000]




UR5.MoveJ(before_2nd_press, blocking=True)
time.sleep(1)
UR5.MoveL(second_press, blocking=True)
UR5.MoveL(before_2nd_press, blocking=True)

#wait 15 seconds
time.sleep(15)

safe = [-36.630000, -109.890000, -117.470000, -125.300000, -69.610000, -220.760000]
UR5.MoveL(safe, blocking=True)

UR5.MoveJ(J_intermediatepoint2, blocking=True)

time.sleep(1)

#UR5.MoveL(T_xos,blocking=True)


# detach mazzer tool
#tls.mazzer_tool_detach_r_ati()

# go back home
#UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)





