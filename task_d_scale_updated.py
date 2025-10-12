# -*- coding: utf-8 -*-
"""
Integrated RoboDK–Scale control (Robot 1)
UR5_R1 waits until the Rancilio scale (Robot 1) reads 20g, then returns to T_loc.
"""

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm
import tools
from modbus_scale_client import modbus_scale_client  # from robodk_scales.py

# ================================
# RoboDK Setup
# ================================
RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

# ================================
# Scale Setup (Robot 1)
# ================================
IP_RANCILIO_1 = "192.168.20.4"
client = modbus_scale_client.ModbusScaleClient(host=IP_RANCILIO_1)

if not client.server_exists():
    RDK.ShowMessage("⚠ No scale detected — output will be simulated (Robot 1).")
    simulate = True
else:
    RDK.ShowMessage("✅ Scale connected (Robot 1).")
    simulate = False

# ================================
# Define Joint Positions & Matrices
# ================================
J_intermediatepoint1 = [-39.78, -117.35, -109.39, -125.3, -69.61, -40.31]
J_intermediatepoint2 = [-27.85, -89.5, -139.23, -125.3, -69.61, -40.31]
J_intermediatepoint3 = [0.0, -79.56, -151.16, -87.51, -23.87, 0.0]

UR_T_MF = np.array([
    [-0.87898362, 0.1756593, -0.44331885, 502],
    [-0.47685197, -0.32379368, 0.81717187, -419.6],
    [0., 0.92967815, 0.36837282, 318.6],
    [0, 0, 0, 1]
])

MT_T_ML = np.array([
    [1, 0, 0, -50],
    [0, 1, 0, 0],
    [0, 0, 1, 67.06],
    [0, 0, 0, 1]
])

TCP_T_MT = np.array([
    [np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
    [np.sin(-50 * np.pi / 180),  np.cos(-50 * np.pi / 180), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

MF_T_ML_loc = np.array([
    [-0.87898362, 0.47685197, 0., 160],
    [-0.17476683, -0.32214857, 0.93041757, -131.3],
    [0.44367145, 0.8178218, 0.36650123, -140],
    [0., 0., 0., 1.]
])

MF_T_ML_pull = np.array([
    [-0.87898362, 0.47685197, 0., 120],
    [-0.17476683, -0.32214857, 0.93041757, -131.3],
    [0.44367145, 0.8178218, 0.36650123, 0],
    [0., 0., 0., 1.]
])

MF_T_ML_out = np.array([
    [-0.87898362, 0.47685197, 0., 200],
    [-0.17476683, -0.32214857, 0.93041757, -131.3],
    [0.44367145, 0.8178218, 0.36650123, -58],
    [0., 0., 0., 1.]
])

MF_T_ML_loc_out = np.array([
    [-0.87898362, 0.47685197, 0., 200],
    [-0.17476683, -0.32214857, 0.93041757, -131.3],
    [0.44367145, 0.8178218, 0.36650123, -140],
    [0., 0., 0., 1.]
])

# Apply 90° rotation to align Mazzer frames
R = np.array([
    [1, 0, 0, 0],
    [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)), 0],
    [0, np.sin(np.deg2rad(90)), np.cos(np.deg2rad(90)), 0],
    [0, 0, 0, 1]
])
MF_T_ML_loc = MF_T_ML_loc @ R
MF_T_ML_pull = MF_T_ML_pull @ R
MF_T_ML_out = MF_T_ML_out @ R
MF_T_ML_loc_out = MF_T_ML_loc_out @ R

# Compute transforms (base to TCP)
UR_T_TCP_loc = UR_T_MF @ MF_T_ML_loc @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_pull = UR_T_MF @ MF_T_ML_pull @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_out = UR_T_MF @ MF_T_ML_out @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
UR_T_TCP_loc_out = UR_T_MF @ MF_T_ML_loc_out @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)


# Add 180° rotation about TCP Z
Rz180 = np.array([
    [np.cos(np.deg2rad(180)), -np.sin(np.deg2rad(180)), 0, 0],
    [np.sin(np.deg2rad(180)),  np.cos(np.deg2rad(180)), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

UR_T_TCP_loc_rotated = UR_T_TCP_loc @ Rz180
UR_T_TCP_pull_rotated = UR_T_TCP_pull @ Rz180
UR_T_TCP_out_rotated = UR_T_TCP_out @ Rz180
UR_T_TCP_loc_out = UR_T_TCP_loc_out @ Rz180

T_loc = rm.Mat(UR_T_TCP_loc_rotated.tolist())
T_pull = rm.Mat(UR_T_TCP_pull_rotated.tolist())
T_out = rm.Mat(UR_T_TCP_out_rotated.tolist())
T_loc_out = rm.Mat(UR_T_TCP_loc_out.tolist())

# ================================
# Robot Motion Sequence
# ================================
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

tls.mazzer_tool_attach_r_ati()

UR5.MoveJ(J_intermediatepoint2, blocking=True)
UR5.MoveJ(J_intermediatepoint1, blocking=True)
sleep(1)

UR5.MoveJ(T_loc, blocking=True)
sleep(1)

UR5.MoveJ(T_pull, blocking=True)
sleep(1)




# # ================================
# # Wait for Scale Fill (20 g)
# # ================================
target_weight = 20.0
current_weight = 0.0

#RDK.ShowMessage("Waiting for Robot 1 scale to reach %.1f g..." % target_weight)

while current_weight < target_weight:
    if simulate:
        current_weight += 2.5  # simulate fill rate
    else:
        current_weight = client.read()
    #RDK.ShowMessage(f"[Robot 1] Current weight: {current_weight:.2f} g")
    sleep(0.5)

UR5.MoveJ(T_out, blocking=True)
sleep(1)

RDK.ShowMessage("✅ Robot 1 scale reached %.1f g. Returning to T_loc..." % current_weight)
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
RDK.ShowMessage("☕ Robot 1 cycle complete.")
