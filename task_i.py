# -*- coding: utf-8 -*-
"""
Created on Sun Oct 12 15:18:10 2025

@author: Owner
"""

####task i#####

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm
import tools
import robodk

RDK = Robolink()
#RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

def taski():


    # DEFINE MATRICES HERE
    J_intermediatepoint1 = [0.000000, -85.520000, -151.160000, -119.340000, -90.000000, 140.000000]
    J_intermediatepoint2 = [-69.610000, -61.660000, -117.350000, -89.500000, 95.470000, 0.000000]
    J_intermediatepoint3 = [-69.610000, -63.650000, -151.160000, -89.500000, -93.480000, 0.000000]

    # Define Radius

    rad = 30

    # Define transforms
    UR_T_WDT = np.array([[-1, 0, 0, 590.2],
                         [0, -1, 0, -100.7],
                         [0, 0, 1, 86.5],
                         [0, 0, 0, 1]])

    WDT_T_RF = np.array([[1, 0, 0, 35],
                         [0, 1, 0, 1.4],
                         [0, 0, -1, 84.2],
                         [0, 0, 0, 1]])

    WDT_T_RF_mid = np.array([[1, 0, 0, 28 - rad],
                             [0, 1, 0, 1.4 + rad],
                             [0, 0, -1, 84.2],
                             [0, 0, 0, 1]])

    WDT_T_RF_mid2 = np.array([[1, 0, 0, 28 - rad],
                              [0, 1, 0, 1.4 - rad],
                              [0, 0, -1, 84.2],
                              [0, 0, 0, 1]])

    WDT_T_RF_end = np.array([[1, 0, 0, 28 - 2 * rad],
                             [0, 1, 0, 1.4],
                             [0, 0, -1, 84.2],
                             [0, 0, 0, 1]])

    MT_T_PB = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 106],
                        [0, 0, 0, 1]])

    TCP_T_MT = np.array([[np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
                         [np.sin(-50 * np.pi / 180), np.cos(-50 * np.pi / 180), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    # Compute base-to-TCP transform (unrotated)
    UR_T_TCP = UR_T_WDT @ WDT_T_RF @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
    UR_T_TCP_mid = UR_T_WDT @ WDT_T_RF_mid @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
    UR_T_TCP_mid2 = UR_T_WDT @ WDT_T_RF_mid2 @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)
    UR_T_TCP_end = UR_T_WDT @ WDT_T_RF_end @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)

    # Convert numpy array into a RoboDK matrix
    T_0 = rm.Mat(UR_T_TCP.tolist())
    T = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0))
    T_0_mid = rm.Mat(UR_T_TCP_mid.tolist())
    T_mid = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_mid))
    T_0_mid2 = rm.Mat(UR_T_TCP_mid2.tolist())
    T_mid2 = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_mid2))
    T_0_end = rm.Mat(UR_T_TCP_end.tolist())
    T_end = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_end))

    # Reset the sim
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

    # Pick up mazzer tool
    tls.mazzer_tool_attach_r_ati()

    time.sleep(1)

    # Move to rotater faster position
    UR5.MoveJ(T, blocking=True)

    UR5.MoveC(T_mid, T_end)
    UR5.MoveC(T_mid2, T)

    time.sleep(0.5)

    UR5.MoveC(T_mid, T_end)
    UR5.MoveC(T_mid2, T)

    time.sleep(0.5)

    UR5.MoveC(T_mid, T_end)
    UR5.MoveC(T_mid2, T)

    time.sleep(0.5)

    UR5.MoveC(T_mid, T_end)
    UR5.MoveC(T_mid2, T)

    time.sleep(0.5)

    UR5.MoveC(T_mid, T_end)
    UR5.MoveC(T_mid2, T)

    # ------------------------------------------------------------

    time.sleep(1)
    tls.mazzer_tool_detach_r_ati()

    # Return home
    # UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)


#taski()