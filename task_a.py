# -*- coding: utf-8 -*-
"""
Created on Mon Sep 22 12:09:02 2025

@author: fsmcn
"""

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm
import robodk
import tools

RDK = Robolink()
#RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

def taska():


    #DEFINE MATRICES HERE
    # define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
    J_intermediatepoint_1 = [-8.840000, -84.630000, -145.260000, -135.160000, -44.210000, -221.05]
    #J_intermediatepoint_1 = [-18.814643, -106.263000, -127.579643, -125.504571, -33.759643, 139.000000]
    J_intermediatepoint_2 = [-7.332113, -106.007468, -126.225228, -123.837868, -36.251128, -221.05]
    #J_intermediatepoint_3 = [143.035341, -76.864448, 132.479957, -58.497983, 112.188127, 139.269728]



    #Define the position where the rancilio tool sits on the mazzer scale 

    offsetleft= 3.5
    offsetforward = -2.7
    offsetvert = -4
    
    UR_T_MZS = np.array([[ 0.5,     0.86666,    0,   440.7 ],
                    [     -0.8666,      0.5,    0,  -275.3 ],
                    [              0,               0,          1,   42.4 ],
                    [              0,               0,    0,         1.000000 ]])

    MZS_T_BBA = np.array([[             0,            0,    1,   -10.6 +offsetforward ],
                    [               0,              -1 ,    0,  -21.4 +offsetleft],
                    [              1,               0,     0,   14.2 +offsetvert ],
                    [              0,               0,    0,         1.000000 ]])


    MZS_T_BBA_eject = np.array([[             0,            0,    1,   -10.6 +offsetforward - 60 ],
                    [               0,              -1 ,    0,  -21.4 +offsetleft],
                    [              1,               0,     0,   14.2 +offsetvert ],
                    [              0,               0,    0,         1.000000 ]])


    RCT_T_BBI = np.array([[             1,            0,    0,   -32 ],
                    [               0,              1 ,    0,  0 ],
                    [              0,               0,     1,   28.07 ],
                    [              0,               0,    0,         1.000000 ]])

    TCP_T_RCT = np.array([[ np.cos(-50 * np.pi/180),     -np.sin(-50 * np.pi/180),    0, 0 ],
                    [ np.sin(-50 * np.pi/180),     np.cos(-50 * np.pi/180),    0,  0 ],
                    [  0,                  0,   1, 0 ],
                    [  0.000000,     0.000000,     0.000000,    1.000000 ]])
        


    UR_T_TCP = UR_T_MZS @ MZS_T_BBA @ np.linalg.inv(RCT_T_BBI) @ np.linalg.inv(TCP_T_RCT )

    UR_T_TCP_e = UR_T_MZS @ MZS_T_BBA_eject @ np.linalg.inv(RCT_T_BBI) @ np.linalg.inv(TCP_T_RCT )
                            
    # convert numpy array into an RDK matrix
    T0 = rm.Mat(UR_T_TCP.tolist())
    T = robodk.UR_2_Pose(robodk.Pose_2_UR(T0))

    T0_eject = rm.Mat(UR_T_TCP_e.tolist())
    T_e = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_eject))


    # reset the sim
    robot_program = RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

    # pick up rancilio tool
    tls.rancilio_tool_attach_l_ati()

    penis = [101.720000, -107.490000, 134.910000, -9.380000, 110.300000, 139.660000]
    UR5.MoveJ(penis, blocking=True)

    # joint movement using a joint array
    int_int_flip = [64.420000, -114.940000, 127.580000, -8.840000, 114.940000, 137.680000]

    UR5.MoveJ(int_int_flip, blocking=True)

    int_flip = [137.680000, -114.950000, 127.580000, -8.840000, 114.950000, 137.680000]
    UR5.MoveJ(int_flip, blocking=True)
    flip = [145.260000, -82.110000, 140.210000, -61.890000, 114.950000, 137.680000]
    UR5.MoveJ(flip, blocking=True)

    raised = [141.180000, -77.050000, 130.140000, -55.770000, 110.860000, 139.410000]
    UR5.MoveL(raised, blocking=True)

    #UR5.MoveJ(J_intermediatepoint_1, blocking=True)
    #UR5.MoveJ(J_intermediatepoint_2, blocking=True)

    time.sleep(1)
    #UR5.MoveJ(J_intermediatepoint_3, blocking=True)

    # linear movement using an HT matrix
    UR5.MoveL(T, blocking=True)

    time.sleep(1)
    tls.student_tool_detach()
    time.sleep(1)
    UR5.MoveL(T_e, blocking=True)


    UR5.MoveJ(int_int_flip, blocking=True)

    #UR5.MoveJ(J_intermediatepoint_1, blocking=True)
    # go back home
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)


#taska()




