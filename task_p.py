import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm

import tools
import robodk
import math
from modbus_scale_client import modbus_scale_client  # from robodk_scales.py


RDK = Robolink()
#RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

def taskp():

    J_intermediatepoint = [-127.520000, -105.120000, -146.060000, 71.180000, 98.020000, -40.010000]

    IP_RANCILIO_1 = "192.168.20.4"
    client = modbus_scale_client.ModbusScaleClient(host=IP_RANCILIO_1)

    if not client.server_exists():
        RDK.ShowMessage("⚠ No scale detected — output will be simulated (Robot 1).")
        simulate = True
    else:
        RDK.ShowMessage("✅ Scale connected (Robot 1).")
        simulate = False

    UR_T_CM = np.array([[np.cos(60 * np.pi / 180), -np.sin(60 * np.pi / 180), 0, -413],
                       [np.sin(60 * np.pi / 180), np.cos(60 * np.pi / 180), 0, -479.4],
                       [0, 0, 1, 349.3],
                       [0, 0, 0, 1.000000]])

    CM_T_PB = np.array([[0, 0, -1, 48.4],
                        [0, 1, 0, 37.8],
                        [1, 0, 0, -60],
                        [0, 0, 0, 1.000000]])

    MT_T_PB = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 103],
                        [0, 0, 0., 1]])

    TCP_T_MT = np.array([[np.cos(-50 * np.pi / 180), -np.sin(-50 * np.pi / 180), 0, 0],
                        [np.sin(-50 * np.pi / 180), np.cos(-50 * np.pi / 180), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])


    UR_T_TCP = UR_T_CM @ CM_T_PB @ np.linalg.inv(MT_T_PB) @ np.linalg.inv(TCP_T_MT)

    T0 = rm.Mat(UR_T_TCP.tolist())
    T = robodk.UR_2_Pose(robodk.Pose_2_UR(T0))

    # Create an offset pose relative to the target T
    T_on = T * rm.transl(4, -4, 7)
    T_off = T * rm.transl(-7, 7, 7)


    UR5.MoveJ(J_intermediatepoint, blocking=True)
    time.sleep(1)

    UR5.MoveJ(T, blocking=True)
    time.sleep(1)
    UR5.MoveJ(T_on, blocking=True)
    time.sleep(1)
    UR5.MoveJ(T, blocking=True)
    time.sleep(1)



    target_weight = 32.0
    current_weight = 0.0


    while current_weight < target_weight:
        if simulate:


            current_weight += 2.5  # simulate fill rate
        else:

            current_weight = client.read()

    UR5.MoveJ(T_off, blocking=True)
    time.sleep(1)
    RDK.ShowMessage("✅ Robot 1 scale reached %.1f g. Returning to T_loc..." % current_weight)
    UR5.MoveJ(J_intermediatepoint, blocking=True)

#taskp()