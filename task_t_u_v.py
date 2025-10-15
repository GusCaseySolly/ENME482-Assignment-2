# -*- coding: utf-8 -*-
"""
Created on Thu Oct  9 18:17:09 2025

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
#RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

def tasktuv():


    # DEFINE MATRICES HERE
    # define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)

    J_intermediatepoint_1 = [-97.390000, -80.080000, -139.240000, 39.680000, 87.290000, -217.150000]
    J_intermediatepoint_2 = [-104.240000, -78.640000, -133.470000, -148.090000, -104.680000, -40.390000]
    J_intermediatepoint_3 = [-101.720000, -77.190000, -116.150000, -72.870000, 95.950000, -40.390000]

    # Define the position where the cup tool moves


    UR_T_TCS = np.array([[1, 0, 0, -151.7],
                         [0, 1, 0, -543.9],
                         [0, 0, 1, 180],
                         [0, 0, 0, 1.000000]])

    TCS_T_silicon = np.array([[0, 1, 0, -43],
                              [0, 0, -1, -57],
                              [-1, 0, 0, 2],
                              [0, 0, 0, 1.000000]])

    RCT_T_brush = np.array([[1, 0, 0, 30],
                            [0, 1, 0, 0],
                            [0, 0, 1, 147.8],
                            [0.000000, 0.000000, 0.000000, 1.000000]])

    TCP_T_RCT = np.array([[np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
                          [np.sin(-50 * np.pi / 180), np.cos(-50 * np.pi / 180), 0, 0],
                          [0, 0, 1, 0],
                          [0.000000, 0.000000, 0.000000, 1.000000]])

    UR_T_TCP = UR_T_TCS @ TCS_T_silicon @ np.linalg.inv(RCT_T_brush) @ np.linalg.inv(TCP_T_RCT)

    T0 = rm.Mat(UR_T_TCP.tolist())
    T = robodk.UR_2_Pose(robodk.Pose_2_UR(T0))

    TCS_T_silicon_engage = np.array([[0, 1, 0, -43],
                                     [0, 0, -1, -57],
                                     [-1, 0, 0, 8.2 - 18],
                                     [0, 0, 0, 1.000000]])

    UR_T_TCP_eng = UR_T_TCS @ TCS_T_silicon_engage @ np.linalg.inv(RCT_T_brush) @ np.linalg.inv(TCP_T_RCT)

    T0_eng = rm.Mat(UR_T_TCP_eng.tolist())
    T_eng = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_eng))
    # ---------------------------------------------------------------------------------------------------------------


    #UR5.MoveJ(J_intermediatepoint_1, blocking=True)

    UR5.MoveJ(T, blocking=True)
    UR5.MoveL(T_eng, blocking=True)
    time.sleep(5)
    UR5.MoveL(T, blocking=True)
    UR5.MoveJ(J_intermediatepoint_2, blocking=True)

    # -----------------------------------------------------------------------------------------


    TCS_T_bristle = np.array([[0, 1, 0, 47],
                              [0, 0, -1, -55],
                              [-1, 0, 0, 8.2],
                              [0, 0, 0, 1.000000]])

    TCS_T_bristle_engage = np.array([[0, 1, 0, 47],
                                     [0, 0, -1, -55],
                                     [-1, 0, 0, 8.2 - 18],
                                     [0, 0, 0, 1.000000]])

    UR_T_TCP_2 = UR_T_TCS @ TCS_T_bristle @ np.linalg.inv(RCT_T_brush) @ np.linalg.inv(TCP_T_RCT)

    T0_2 = rm.Mat(UR_T_TCP_2.tolist())
    T_2 = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_2))

    UR_T_TCP_2_eng = UR_T_TCS @ TCS_T_bristle_engage @ np.linalg.inv(RCT_T_brush) @ np.linalg.inv(TCP_T_RCT)

    T0_eng_2 = rm.Mat(UR_T_TCP_2_eng.tolist())
    T_eng_2 = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_eng_2))
    #-----------------------------------------------------------------------------------------------------------------

    UR5.MoveJ(T_2, blocking=True)
    UR5.MoveL(T_eng_2, blocking=True)
    time.sleep(5)
    UR5.MoveL(T_2, blocking=True)
    UR5.MoveJ(J_intermediatepoint_2, blocking=True)

    UR5.MoveJ(J_intermediatepoint_3, blocking=True)

    tls.rancilio_tool_detach_r_ati()


#tasktuv()