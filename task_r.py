# -*- coding: utf-8 -*-
"""
Created on Wed Oct  8 18:29:20 2025

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

def taskr():

    # DEFINE MATRICES HERE
    # define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)

    J_intermediatepoint_1 = [-91.620000, -69.980000, -153.670000, -136.350000, -44.010000, -221.480000]
    J_intermediatepoint_2 = [-133.260000, -74.310000, -136.350000, -149.340000, -103.710000, -220.340000]
    J_intermediatepoint_3 = [-28.120000, -104.880000, -113.020000, -47.960000, 90.000000, -6.840000]

    # Define the position where the cup tool moves


    dUR_T_RS = np.array([[0.5, -0.86666, 0, -355.6],
                         [0.8666, 0.5, 0, -330.5],
                         [0, 0, 1, 42.5],
                         [0, 0, 0, 1.000000]])

    UR_T_RS = np.array([[np.cos(-120 * np.pi / 180), -np.sin(-120 * np.pi / 180), 0, -385.6],
                        [np.sin(-120 * np.pi / 180), np.cos(-120 * np.pi / 180), 0, -330.5],
                        [0, 0, 1, 42.5 + 10],
                        [0, 0, 0, 1.000000]])

    RS_T_cup = np.array([[0, 0, 1, 157.72],
                         [0, -1, 0, -19],
                         [1, 0, 0, 24.24 + 10],
                         [0, 0, 0, 1.000000]])

    RS_T_cup_up = np.array([[0, 0, 1, 157.72],
                            [0, -1, 0, -19],
                            [1, 0, 0, 24.24 + 30],
                            [0, 0, 0, 1.000000]])

    CT_T_cup = np.array([[1, 0, 0, -104.5],
                         [0, 1, 0, 0],
                         [0, 0, 1, 186.62],
                         [0.000000, 0.000000, 0.000000, 1.000000]])

    TCP_T_CT = np.array([[np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
                         [np.sin(-50 * np.pi / 180), np.cos(-50 * np.pi / 180), 0, 0],
                         [0, 0, 1, 0],
                         [0.000000, 0.000000, 0.000000, 1.000000]])

    UR_T_TCP = UR_T_RS @ RS_T_cup @ np.linalg.inv(CT_T_cup) @ np.linalg.inv(TCP_T_CT)

    UR_T_TCP_up = UR_T_RS @ RS_T_cup_up @ np.linalg.inv(CT_T_cup) @ np.linalg.inv(TCP_T_CT)

    # convert numpy array into an RDK matrix
    T_0 = rm.Mat(UR_T_TCP.tolist())

    T = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0))

    T_0_up = rm.Mat(UR_T_TCP_up.tolist())

    T_up = robodk.UR_2_Pose(robodk.Pose_2_UR(T_0_up))

    # reset the sim
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

    # pick up rancilio tool
    tls.cup_tool_attach_r_ati()

    # joint movement using a joint array

    UR5.MoveJ(J_intermediatepoint_1, blocking=True)

    UR5.MoveJ(J_intermediatepoint_2, blocking=True)

    time.sleep(1)
    # linear movement using an HT matrix
    tls.cup_tool_open_ur5()
    UR5.MoveL(T, blocking=True)

    tls.cup_tool_shut_ur5()
    # open cup tool


    UR5.MoveL(T_up, blocking=True)

    final_coffee = [-146.450000, -90.180000, -137.800000, -130.580000, -20.910000, -220.360000]
    not_final_coffee = [-146.440000, -71.420000, -136.350000, -150.780000, -20.910000, -220.370000]
    mid_coffee = [-116.150000, -78.640000, -139.240000, -139.240000, -20.920000, -220.370000]
    retract = [-108.940000, -71.420000, -146.450000, -143.570000, 13.710000, -220.360000]
    fake_out = [-146.440000, -80.080000, -137.800000, -130.580000, -20.910000, -205.610000]

    time.sleep(1)

    UR5.MoveJ(mid_coffee, blocking=True)
    UR5.MoveL(not_final_coffee, blocking=True)

    UR5.MoveJ(fake_out, blocking = True)

    UR5.MoveJ(final_coffee, blocking=True)
    time.sleep(1)



    tls.cup_tool_open_ur5()

    UR5.MoveJ(retract, blocking=True)
    # go back home
    tls.cup_tool_detach_r_ati()

#taskr()