# -*- coding: utf-8 -*-
"""
Created on Sat Oct 11 12:59:46 2025

@author: Owner
"""

#### task g ###

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
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

# DEFINE MATRICES HERE
# define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
J_intermediatepoint1 = [0.000000, -109.390000, -157.130000, 71.600000, 87.510000, -33.810000]
J_intermediatepoint2 = [-27.850000, -89.500000, -139.230000, -125.300000, -69.610000, -40.310000]
J_intermediatepoint3 = [0.000000, -79.560000, -151.160000, -87.510000, -23.870000, 0.000000]

# Define the position where the mazzer tool slots into the cup dispenser index
UR_T_WDT = np.array([[-1,  0,  0, 590.2],
                    [0, -1,  0, -100.7],
                    [ 0.,          0,   1, 86.5],
                    [0, 0, 0, 1]])

WDT_T_CDH = np.array([[-1, 0, 0, 108],
                    [0, 1, 0, 0],
                    [0, 0, 1, -42.3],
                    [0, 0, 0, 1]])

RT_T_BBI = np.array([[1, 0, 0, -32],
                     [0,  1, 0, 0],
                     [0, 0, 1, 28.07],
                     [0, 0, 0, 1]])

TCP_T_RT = np.array([[ np.cos(-50 * np.pi/180),     -np.sin(-50 * np.pi/180),    0, 0 ],
                 [ np.sin(-50 * np.pi/180),     np.cos(-50 * np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])


# Compute base-to-TCP transform (unrotated)
UR_T_TCP = UR_T_WDT @ WDT_T_CDH @ np.linalg.inv(RT_T_BBI) @ np.linalg.inv(TCP_T_RT)


# Convert numpy array into a RoboDK matrix
T = rm.Mat(UR_T_TCP.tolist())



# Reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# Pick up mazzer tool



tls.rancilio_tool_attach_r_ati()

# Move to intermediate points
#UR5.MoveJ(J_intermediatepoint2, blocking=True)
#UR5.MoveJ(J_intermediatepoint1, blocking=True)

time.sleep(1)


UR5.MoveJ(T, blocking=True)

time.sleep(1)

# Detach mazzer tool
tls.mazzer_tool_detach_r_ati()

# Go back home
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

