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
J_intermediatepoint1 = [-39.780000, -117.350000, -109.390000, -125.300000, -69.610000, -40.310000]
J_intermediatepoint2 = [-27.850000, -89.500000, -139.230000, -125.300000, -69.610000, -40.310000]
J_intermediatepoint3 = [0.000000, -79.560000, -151.160000, -87.510000, -23.870000, 0.000000]

# Define the position where the mazzer tool slots into the cup dispenser index
UR_T_MF = np.array([[-0.87898362,  0.1756593,  -0.44331885, 502],
                    [-0.47685197, -0.32379368,  0.81717187, -419.6],
                    [ 0.,          0.92967815,   0.36837282, 318.6],
                    [0, 0, 0, 1]])

MT_T_ML = np.array([[1, 0, 0, -50],
                    [0, 1, 0, 0],
                    [0, 0, 1, 67.06],
                    [0, 0, 0, 1]])

TCP_T_MT = np.array([[np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
                     [np.sin(-50 * np.pi / 180),  np.cos(-50 * np.pi / 180), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

MF_T_ML_loc = np.array([[-0.87898362,  0.47685197,  0.,        160],
                    [-0.17476683, -0.32214857,  0.93041757, -131.3],
                    [0.44367145,  0.8178218,   0.36650123,  -140],
                    [0.,          0.,          0.,          1.]])

MF_T_ML_pull = np.array([[-0.87898362,  0.47685197,  0.,        160],
                    [-0.17476683, -0.32214857,  0.93041757, -131.3],
                    [0.44367145,  0.8178218,   0.36650123,  -58],
                    [0.,          0.,          0.,          1.]])

MF_T_ML_out = np.array([[-0.87898362,  0.47685197,  0.,        200],
                    [-0.17476683, -0.32214857,  0.93041757, -131.3],
                    [0.44367145,  0.8178218,   0.36650123,  -58],
                    [0.,          0.,          0.,          1.]])

R = np.array([[1, 0, 0, 0],
              [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)), 0],
              [0, np.sin(np.deg2rad(90)),  np.cos(np.deg2rad(90)), 0],
              [0, 0, 0, 1]])

# Overwrite with rotated version
MF_T_ML_loc = MF_T_ML_loc @ R
MF_T_ML_pull = MF_T_ML_pull @ R
MF_T_ML_out = MF_T_ML_out @ R

# Compute base-to-TCP transform (unrotated)
UR_T_TCP_loc = UR_T_MF @ MF_T_ML_loc @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_pull = UR_T_MF @ MF_T_ML_pull @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_out = UR_T_MF @ MF_T_ML_out @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)


# >>> ADD 180° ROTATION ABOUT TCP Z-AXIS <<<
Rz180 = np.array([
    [np.cos(np.deg2rad(180)), -np.sin(np.deg2rad(180)), 0, 0],
    [np.sin(np.deg2rad(180)),  np.cos(np.deg2rad(180)), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Apply 180° rotation for Mazzer lever interaction
UR_T_TCP_loc_rotated = UR_T_TCP_loc @ Rz180
UR_T_TCP_pull_rotated = UR_T_TCP_pull @ Rz180
UR_T_TCP_out_rotated = UR_T_TCP_out @ Rz180
# Convert numpy array into a RoboDK matrix
T_loc = rm.Mat(UR_T_TCP_loc_rotated.tolist())
T_pull = rm.Mat(UR_T_TCP_pull_rotated.tolist())
T_out = rm.Mat(UR_T_TCP_out_rotated.tolist())


# Reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# Pick up mazzer tool
tls.mazzer_tool_attach_r_ati()

# Move to intermediate points
UR5.MoveJ(J_intermediatepoint2, blocking=True)
UR5.MoveJ(J_intermediatepoint1, blocking=True)

time.sleep(1)

# Move to Mazzer lever position (with 180° rotated TCP)
UR5.MoveJ(T_loc, blocking=True)

time.sleep(1)

UR5.MoveJ(T_pull, blocking = True)

time.sleep(1)

UR5.MoveJ(T_out, blocking = True)

time.sleep(1)

UR5.MoveJ(J_intermediatepoint2, blocking=True)

time.sleep(1)

# Detach mazzer tool
tls.mazzer_tool_detach_r_ati()

# Go back home
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

