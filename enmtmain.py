# -*- coding: utf-8 -*-
"""
Created on Thu Sep 18 13:52:46 2025

@author: Owner
"""

import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm

import tools
import robodk
import math

import task_a
import task_b
import task_c
import task_d
import task_e
import task_f
import task_g_h
import task_i
import task_j
import task_k
import task_l
import task_m
import task_n
import task_o
import task_p
import task_q
import task_r
import task_s
import task_t_u_v


RDK = Robolink()
#RDK.setRunMode(RUNMODE_SIMULATE)
RDK.setRunMode(RUNMODE_RUN_ROBOT)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

#task_a.taska()
#task_b.taskb()
#task_c.taskc()
#task_d.taskd()
#task_e.taske()
task_f.taskf()
task_g_h.taskgh()
task_i.taski()
task_m.taskm()
task_n.taskn()
task_j.taskj()
task_k.taskk()
task_l.taskl()
task_o.tasko()
task_p.taskp()
task_q.taskq()
task_s.tasks()
task_t_u_v.tasktuv()
task_r.taskr()
