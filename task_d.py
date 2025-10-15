# -*- coding: utf-8 -*-
"""
Integrated RoboDK–Scale control (Robot 1)
UR5_R1 waits until the Rancilio scale (Robot 1) reads 20g, then returns to T_loc.
MF_T_ML matrix rotated 21.3° clockwise about x-axis.
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
#RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

def taskd():


    # ================================
    # Scale Setup (Robot 1)
    # ================================
    IP_MAZZER_1 = "192.168.20.3"
    client = modbus_scale_client.ModbusScaleClient(host=IP_MAZZER_1)

    if not client.server_exists():
        RDK.ShowMessage("⚠ No scale detected — output will be simulated (Robot 1).")
        simulate = True
    else:
        #RDK.ShowMessage("✅ Scale connected (Robot 1).")
        simulate = False

    # ================================
    # Define Joint Positions & Matrices
    # ================================
    J_intermediatepoint1 = [-9.940000, -75.580000, -103.430000, -119.020000, 63.650000, -173.840000]
    J_intermediatepoint2 = [-39.940000, -93.480000, -125.300000, -157.130000, -77.570000, -130.230000]
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
        [np.sin(-50 * np.pi / 180), np.cos(-50 * np.pi / 180), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    MF_T_ML_near = np.array([
        [1, 0, 0., 130],
        [0, -1, 0, -154],
        [0, 0, -1, -45],
        [0., 0., 0., 1.]
    ])

    MF_T_ML_loc = np.array([
        [1, 0, 0., 72],
        [0, -1, 0, -154],
        [0, 0, -1, -45],
        [0., 0., 0., 1.]
    ])

    MF_T_ML_mid = np.array([
        [1, 0, 0., 84.905],
        [0, -1, 0, -154],
        [0, 0, -1, 0],
        [0., 0., 0., 1.]
    ])

    MF_T_ML_end = np.array([
        [1, 0, 0., 74.96  - 10],
        [0, -1, 0, -154 ],
        [0, 0, -1, 39.86 + 10],
        [0., 0., 0., 1.]
    ])

    MF_T_ML_out = np.array([
        [1, 0, 0., 130],
        [0, -1, 0, -154],
        [0, 0, -1, 39.86],
        [0., 0., 0., 1.]
    ])

    theta = 21.3 * np.pi / 180  # negative for clockwise
    R_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

    MF_T_ML_near_rotated = R_x @ MF_T_ML_near
    MF_T_ML_loc_rotated = R_x @ MF_T_ML_loc
    MF_T_ML_mid_rotated = R_x @ MF_T_ML_mid
    MF_T_ML_end_rotated = R_x @ MF_T_ML_end
    MF_T_ML_out_rotated = R_x @ MF_T_ML_out

    # ================================
    # Compute transforms (base to TCP)
    # ================================
    UR_T_TCP_near = UR_T_MF @ MF_T_ML_near_rotated @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
    UR_T_TCP_loc = UR_T_MF @ MF_T_ML_loc_rotated @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
    UR_T_TCP_mid = UR_T_MF @ MF_T_ML_mid_rotated @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
    UR_T_TCP_end = UR_T_MF @ MF_T_ML_end_rotated @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)
    UR_T_TCP_out = UR_T_MF @ MF_T_ML_out_rotated @ np.linalg.inv(MT_T_ML) @ np.linalg.inv(TCP_T_MT)

    T_near = rm.Mat(UR_T_TCP_near.tolist())
    T_loc = rm.Mat(UR_T_TCP_loc.tolist())
    T_mid = rm.Mat(UR_T_TCP_mid.tolist())
    T_end = rm.Mat(UR_T_TCP_end.tolist())
    T_out = rm.Mat(UR_T_TCP_out.tolist())

    # ================================
    # Robot Motion Sequence
    # ================================
    # robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    # robot_program.RunCode()
    # robot_program.WaitFinished()

    #tls.mazzer_tool_attach_r_ati() #REMOVE THIS FOR FINAL---------------------------------hjsfdjksdjsdghjsDGHghjssesgsghsgdzhjsgdjhz

    UR5.MoveJ(J_intermediatepoint1, blocking=True)

    # ================================
    # Wait for Scale Fill (20 g)
    # ================================
    target_weight = 20.0
    current_weight = 0.0

    while current_weight < target_weight:
        if simulate:

            UR5.MoveJ(J_intermediatepoint2, blocking=True)

            UR5.MoveJ(T_near, blocking=True)

            sleep(1)

            UR5.MoveL(T_loc, blocking=True)

            sleep(1)

            UR5.MoveC(T_mid, T_end, blocking=True)

            sleep(1)

            UR5.MoveL(T_out, blocking=True)

            current_weight += 20  # simulate fill rate
        else:
            UR5.MoveJ(J_intermediatepoint2, blocking=True)

            UR5.MoveJ(T_near, blocking=True)

            sleep(1)

            UR5.MoveL(T_loc, blocking=True)

            sleep(1)

            UR5.MoveC(T_mid, T_end, blocking=True)

            sleep(1)

            UR5.MoveL(T_out, blocking=True)

            current_weight = client.read()
        sleep(0.5)

    UR5.MoveJ(J_intermediatepoint2, blocking=True)

    UR5.MoveJ(J_intermediatepoint1, blocking=True)

    #tls.mazzer_tool_detach_r_ati()

    #RDK.ShowMessage("✅ Robot 1 scale reached %.1f g. Returning to T_loc..." % current_weight)
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
    #RDK.ShowMessage("☕ Robot 1 cycle complete.")

    
#taskd()