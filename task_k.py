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

def taskk():

    # DEFINE MATRICES HERE
    # define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
    J_intermediatepoint = [-18.814643, -106.263000, -127.579643, -125.504571, -33.759643, 139.000000]

    # Define the position where the mazzer tool slots into the cup dispensor index

    UR_T_PQ = np.array([[np.cos(-135 * np.pi / 180), -np.sin(-135 * np.pi / 180),  0, 387.4],
                        [np.sin(-135 * np.pi / 180), np.cos(-135 * np.pi / 180),  0, 85.3],
                        [ 0.000000,  0.000000,  1,  271.2],
                        [ 0.000000,  0.000000,  0,    1]])

    #------

    PQ_T_BUC_pre = np.array([[0, 0, -1, 11.6],
                        [0, 1, 0, -0.7],
                        [1, 0, 0, -134],
                        [0, 0, 0., 1]])

    fix = -2.4

    tilt = np.array([[np.cos(fix * np.pi / 180), 0, -np.sin(fix * np.pi / 180), 0],
                     [0, 1, 0, 0],
                     [np.sin(fix * np.pi / 180), 0, np.cos(fix * np.pi / 180), 0],
                     [0, 0, 0, 1.000000]])

    PQ_T_BUC = PQ_T_BUC_pre @ tilt

    #-----

    # define horizontal offset distance (mm) along PQ x-axis
    horizontal_offset = 100  # adjust as needed

    # compute intermediate PQ -> BUC pose
    PQ_T_BUC_intermediate = PQ_T_BUC @ np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -horizontal_offset],
        [0, 0, 0, 1]
    ])

    RCT_T_BUC = np.array([[1, 0, 0, 30],
                          [0, 1, 0, 0],
                          [0, 0, 1, 147.8],
                          [0, 0, 0, 1.000000]])

    TCP_T_RCT = np.array([[np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
                          [np.sin(-50 * np.pi / 180), np.cos(-50 * np.pi / 180), 0, 0],
                          [0, 0, 1, 0],
                          [0.000000, 0.000000, 0.000000, 1.000000]])

    UR_T_TCP = UR_T_PQ @ PQ_T_BUC @ np.linalg.inv(RCT_T_BUC) @ np.linalg.inv(TCP_T_RCT)
    UR_T_TCP_intermediate = UR_T_PQ @ PQ_T_BUC_intermediate @ np.linalg.inv(RCT_T_BUC) @ np.linalg.inv(TCP_T_RCT)

    # convert numpy array into an RDK matrix
    T0 = rm.Mat(UR_T_TCP.tolist())
    T = robodk.UR_2_Pose(robodk.Pose_2_UR(T0))
    T0_intermediate = rm.Mat(UR_T_TCP_intermediate.tolist())
    T_intermediate = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_intermediate))

    # reset the sim
    #robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    #robot_program.RunCode()
    #robot_program.WaitFinished()

    
   

    #UR5.MoveJ(J_intermediatepoint, blocking=True)
    #time.sleep(1)

    UR5.MoveL(T_intermediate, blocking=True)
    #time.sleep(1)

    UR5.MoveJ(T, blocking=True)
    time.sleep(2)

    UR5.MoveL(T_intermediate, blocking=True)
    #time.sleep(1)

#taskk()
