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

def taskn():

    # DEFINE MATRICES HERE
    # define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
    J_intermediatepoint = [-113.370000, -85.520000, -131.270000, -145.190000, -23.870000, -217.450000]
    J_intermediatepoint2 = [-169.030000, -61.190000, -98.540000, -106.340000, -198.050000, -220.630000]

    offset = -25

    # Define the position where the mazzer tool slots into the cup dispensor index

    UR_T_CD = np.array([[0, 0, -1, -590.8],
                        [0, 1, 0, -220.3],
                        [1, 0, 0, 214.3],
                        [0, 0, 0, 1.000000]])

    CD_T_PB = np.array([[1, 0, 0, -153.4],
                        [0, 1, 0, 71],
                        [0, 0, 1, 31.2],
                        [0.000000, 0.000000, 0.000000, 1.000000]])

    CT_T_PB = np.array([[1, 0, 0, -104.5],
                        [0, 1, 0, 0],
                        [0, 0, 1, 186.62],
                        [0.000000, 0.000000, 0.000000, 1.000000]])

    CT_T_PB_alt = np.array([[1, 0, 0, -104.5 - offset],
                        [0, 1, 0, 0],
                        [0, 0, 1, 186.62],
                        [0.000000, 0.000000, 0.000000, 1.000000]])

    TCP_T_CT = np.array([[np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
                         [np.sin(-50 * np.pi / 180), np.cos(-50 * np.pi / 180), 0, 0],
                         [0, 0, 1, 0],
                         [0.000000, 0.000000, 0.000000, 1.000000]])



    UR_T_MZS = np.array([[0.5, 0.86666, 0, 440.7],
                         [-0.8666, 0.5, 0, -275.3],
                         [0, 0, 1, 42.4],
                         [0, 0, 0, 1.000000]])

    # =================================================================================
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

    RS_T_cup_up_alt = np.array([[0, 0, 1, 157.72],
                            [0, -1, 0, -19],
                            [1, 0, 0, 24.24 + 15],
                            [0, 0, 0, 1.000000]])

    CT_T_cup = np.array([[1, 0, 0, -104.5],
                         [0, 1, 0, 0],
                         [0, 0, 1, 186.62],
                         [0.000000, 0.000000, 0.000000, 1.000000]])

    UR_T_TCP = UR_T_RS @ RS_T_cup @ np.linalg.inv(CT_T_cup) @ np.linalg.inv(TCP_T_CT)
    UR_T_TCP_up = UR_T_RS @ RS_T_cup_up @ np.linalg.inv(CT_T_cup) @ np.linalg.inv(TCP_T_CT)
    UR_T_TCP = UR_T_RS @ RS_T_cup @ np.linalg.inv(CT_T_cup) @ np.linalg.inv(TCP_T_CT)
    UR_T_TCP_up_alt = UR_T_RS @ RS_T_cup_up_alt @ np.linalg.inv(CT_T_cup) @ np.linalg.inv(TCP_T_CT)

    # convert numpy array into an RDK matrix
    T0_cup = rm.Mat(UR_T_TCP.tolist())
    T_cup = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_cup))
    T0_cup_up = rm.Mat(UR_T_TCP_up.tolist())
    T_cup_up = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_cup_up))
    T0_cup_up_alt = rm.Mat(UR_T_TCP_up_alt.tolist())
    T_cup_up_alt = robodk.UR_2_Pose(robodk.Pose_2_UR(T0_cup_up_alt))
    # ======================================================================================================



    UR_T_TCP = UR_T_CD @ CD_T_PB @ np.linalg.inv(CT_T_PB) @ np.linalg.inv(TCP_T_CT)
    UR_T_TCP2 = UR_T_CD @ CD_T_PB @ np.linalg.inv(CT_T_PB) @ np.linalg.inv(TCP_T_CT)
    UR_T_TCP_alt = UR_T_CD @ CD_T_PB @ np.linalg.inv(CT_T_PB_alt) @ np.linalg.inv(TCP_T_CT)

    T = robodk.UR_2_Pose(robodk.Pose_2_UR(rm.Mat(UR_T_TCP.tolist())))
    T_alt = robodk.UR_2_Pose(robodk.Pose_2_UR(rm.Mat(UR_T_TCP_alt.tolist())))



    # Define a horizontal offset distance (in mm)
    offset_distance = -100  # move 100 mm horizontally before approaching
    offset_distance2 = -300
    offset_distance3 = -30
    # Create an offset pose relative to the target T
    T_approach = T * rm.transl(0, 0, offset_distance)
    T_approach_alt = T_alt * rm.transl(0, 0, offset_distance)
    T_cup_out = T_cup_up * rm.transl(0, 0, offset_distance2)


    # reset the sim
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

    tls.cup_tool_attach_r_ati()
    time.sleep(1)
    UR5.MoveJ(J_intermediatepoint, blocking=True)
    time.sleep(1)
    tls.cup_tool_open_ur5()
    time.sleep(1)
    UR5.MoveL(T_approach_alt, blocking=True)
    time.sleep(1)
    UR5.MoveL(T_alt, blocking=True)
    time.sleep(1)
    tls.cup_tool_shut_ur5()
    time.sleep(1)
    UR5.MoveL(T, blocking=True)
    time.sleep(1)
    UR5.MoveL(T_approach, blocking=True)
    time.sleep(1)

    UR5.MoveL(T_cup_up, blocking=True)
    time.sleep(1)
    UR5.MoveL(T_cup_up_alt, blocking=True)
    time.sleep(1)
    tls.cup_tool_open_ur5()
    time.sleep(1)
    UR5.MoveL(T_cup_out, blocking=True)
    time.sleep(1)
    tls.cup_tool_shut_ur5()
    time.sleep(1)
    UR5.MoveJ(J_intermediatepoint2, blocking=True)
    time.sleep(1)


    tls.cup_tool_detach_r_ati()

#taskn()


