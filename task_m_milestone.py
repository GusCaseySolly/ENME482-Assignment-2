# -*- coding: utf-8 -*-
"""
Created on Thu Sep 18 13:52:46 2025

@author: Owner
"""

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm
import robodk
import tools

RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
#RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)


#DEFINE MATRICES HERE
# define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
J_intermediatepoint = [-113.370000, -85.520000, -131.270000, -145.190000, -23.870000, -217.450000]



#Define the position where the mazzer tool slots into the cup dispensor index


UR_T_CD = np.array([[ 0,     0,    -1,   -590.8 ],
                 [     0,      1,    0,  -220.3 ],
                 [   1,               0,    0,   214.3],
                 [              0,               0,    0,   1.000000 ]])

#index pull shut
CD_T_PB = np.array([[ 1,     0,    0, -10.8],
                      [ 0,     1,   0,      70.9],
                      [  0,    0,   1,         -39.3],
                      [  0.000000,     0.000000,     0.000000,    1.000000 ]])

MT_T_PB = np.array([[ 1,     0,    0, 0 ],
                 [ 0,     1,    0,  0 ],
                 [  0,    0,  1, 106     ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

TCP_T_MT =  np.array([[ np.cos(-50* np.pi/180),     -np.sin(-50* np.pi/180),    0, 0 ],
                 [ np.sin(-50* np.pi/180),     np.cos(-50* np.pi/180),    0,  0 ],
                 [  0,                  0,   1, 0 ],
                 [  0.000000,     0.000000,     0.000000,    1.000000 ]])

CD_T_PB_vos = np.array([[ 1,     0,    0, 10.8],
                      [ 0,     1,   0,      70.9],
                      [  0,    0,   1,         -39.3],
                      [  0.000000,     0.000000,     0.000000,    1.000000 ]])

pull_out_variable = 35.1 # how far to pull out cup dispensor thing - calculated from excel
CD_T_PB_hos = np.array([[ 1,     0,    0, -10.8],
                      [ 0,     1,   0,      70.9],
                      [  0,    0,   1,         -39.3-pull_out_variable],
                      [  0.000000,     0.000000,     0.000000,    1.000000 ]])

CD_T_PB_vout = np.array([[ 1,     0,    0, 30.8],
                      [ 0,     1,   0,      70.9],
                      [  0,    0,   1,         -39.3-pull_out_variable],
                      [  0.000000,     0.000000,     0.000000,    1.000000 ]])



UR_T_TCP = UR_T_CD @ CD_T_PB @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)

UR_T_TCP_vos = UR_T_CD @ CD_T_PB_vos @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)

UR_T_TCP_hos = UR_T_CD @ CD_T_PB_hos @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)  

UR_T_TCP_vout = UR_T_CD @ CD_T_PB_vout @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT) 
                        
# convert numpy array into an RDK matrix
T = robodk.UR_2_Pose(robodk.Pose_2_UR(rm.Mat(UR_T_TCP.tolist())))
T_vos = robodk.UR_2_Pose(robodk.Pose_2_UR(rm.Mat(UR_T_TCP_vos.tolist())))
T_hos = robodk.UR_2_Pose(robodk.Pose_2_UR(rm.Mat(UR_T_TCP_hos.tolist())))
T_vout = robodk.UR_2_Pose(robodk.Pose_2_UR(rm.Mat(UR_T_TCP_vout.tolist())))



# reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# pick up mazzer tool
tls.mazzer_tool_attach_r_ati()

# joint movement using a joint array

UR5.MoveJ(J_intermediatepoint, blocking=True)

time.sleep(1)

# linear movement using an HT matrix
UR5.MoveJ(T_vos, blocking=True)

time.sleep(1)

UR5.MoveL(T, blocking = True)

time.sleep(1)

UR5.MoveL(T_hos,blocking=True)

time.sleep(1)
UR5.MoveL(T, blocking = True)
time.sleep(1)
UR5.MoveJ(T_vos, blocking=True)
time.sleep(1)






UR5.MoveJ(J_intermediatepoint, blocking=True)


# go back home
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
