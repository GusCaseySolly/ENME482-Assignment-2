#### task j ###

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
import robodk

RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

# DEFINE MATRICES HERE
# define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
J_intermediatepoint1 = [147.180000, -90.000000, 131.270000, -89.990000, -90.000000, -89.990000]
J_intermediatepoint2 = [179.010000, -90.000000, 75.580000, -89.990000, -90.000000, -89.990000]
J_intermediatepoint3 = [-69.610000, -63.650000, -151.160000, -89.500000, -93.480000, 0.000000]

# Define the position where the mazzer tool slots into the cup dispenser index
UR_T_WDT = np.array([[-1,  0,  0, 590.2],
                    [0, -1,  0, -100.7],
                    [ 0.,          0,   1, 86.5],
                    [0, 0, 0, 1]])

WDT_T_CDH_out = np.array([[0, 0, -1, 250],
                    [0, 1, 0, 0],
                    [1, 0, 0, 0],
                    [0, 0, 0, 1]])

WDT_T_CDH_remove = np.array([[0, 0, -1, 250],
                    [0, 1, 0, 0],
                    [1, 0, 0, -30],
                    [0, 0, 0, 1]])

WDT_T_CDH_above = np.array([[0, 0, -1, 120],
                    [0, 1, 0, 0],
                    [1, 0, 0, 0],
                    [0, 0, 0, 1]])

WDT_T_CDH_set = np.array([[0, 0, -1, 120],
                    [0, 1, 0, 0],
                    [1, 0, 0, -30],
                    [0, 0, 0, 1]])

RT_T_BBI = np.array([[1, 0, 0, -32],
                     [0,  1, 0, 0],
                     [0, 0, 1, 28.07],
                     [0, 0, 0, 1]])

#TCP_T_RT = np.array([[ 1,     0,    0, 0 ],
#                 [ 0,    1,    0,  0 ],
#                 [  0,                  0,   1, 0 ],
#                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

TCP_T_RT = np.array([[ np.cos(-50 * np.pi/180),     -np.sin(-50 * np.pi/180),    0, 0 ],
                 [ np.sin(-50 * np.pi/180),     np.cos(-50 * np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

####new with 2.4 degree rotation
#TCP_T_RT = np.array([
#    [ 0.642222,  0.766044, -0.026908,  0],
#    [-0.765373,  0.642788, -0.032049,  0],
#    [ 0.041888,  0.000000,  0.999121,  0],
#    [ 0.000000,  0.000000,  0.000000,  1]])

# Compute base-to-TCP transform (unrotated)
UR_T_TCP_above = UR_T_WDT @ WDT_T_CDH_above @ np.linalg.inv(RT_T_BBI) @ np.linalg.inv(TCP_T_RT)
UR_T_TCP_set = UR_T_WDT @ WDT_T_CDH_set @ np.linalg.inv(RT_T_BBI) @ np.linalg.inv(TCP_T_RT)
UR_T_TCP_out = UR_T_WDT @ WDT_T_CDH_out @ np.linalg.inv(RT_T_BBI) @ np.linalg.inv(TCP_T_RT)
UR_T_TCP_remove = UR_T_WDT @ WDT_T_CDH_remove @ np.linalg.inv(RT_T_BBI) @ np.linalg.inv(TCP_T_RT)

# Convert numpy array into a RoboDK matrix

T_0_above = rm.Mat(UR_T_TCP_above.tolist())
T_above = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_above))
T_0_set = rm.Mat(UR_T_TCP_set.tolist())
T_set = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_set))
T_0_out = rm.Mat(UR_T_TCP_out.tolist())
T_out = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_out))
T_0_remove = rm.Mat(UR_T_TCP_remove.tolist())
T_remove = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_remove))

# Reset the sim

robot_program = RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

tls.wdt_open()

time.sleep(1)

UR5.MoveJ(J_intermediatepoint2, blocking=True)

time.sleep(1)

UR5.MoveJ(J_intermediatepoint1, blocking=True)

time.sleep(1)

UR5.MoveJ(T_remove, blocking=True)

time.sleep(1)

UR5.MoveJ(T_set,blocking=True)

time.sleep(1)

tls.student_tool_attach()

time.sleep(1)
UR5.MoveL(T_above, blocking=True)

higher =[118.670000, -94.770000, 145.010000, -49.780000, 18.040000, -220.460000]

UR5.MoveJ(higher, blocking=True)
UR5.MoveJ(T_remove, blocking=True)

time.sleep(1)

tls.wdt_shut()

time.sleep(1)

#UR5.MoveJ(J_intermediatepoint1, blocking=True)
#UR5.MoveJ(J_intermediatepoint2, blocking=True)
# Go back home
#UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)



