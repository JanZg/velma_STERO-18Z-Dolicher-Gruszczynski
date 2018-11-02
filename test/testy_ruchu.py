#!/usr/bin/env python2

## Runs test for jnt_imp mode motion and planning.
# @ingroup integration_tests
# @file test_jimp_planning.py
# @namespace scripts.test_jimp_planning Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    # define some configurations
    q_map_1 = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
        'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

    rospy.init_node('test_jimp_planning', anonymous=False)

    rospy.sleep(0.5)

    print "This test/tutorial executes complex motions"\
        " in Joint Impedance mode. Planning is used"\
        " in this example.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(3)
    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Also, head motors must be homed after start-up of the robot."
    print "Sending head pan motor START_HOMING command..."
    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(14)
    print "Head pan motor homing successful."

    print "Sending head tilt motor START_HOMING command..."
    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(15)
    print "Head tilt motor homing successful."

    print "Pobieranie pozycji drugiego stolu oraz wyznaczanie kata obrotu"
    T_B_Table = velma.getTf("B", "table_2")
    (rotX, rotY, rotZ) = T_B_Table.M.GetRPY()
    # Wyznaczanie kata obrotu stolu (stol jes symetryczny wiec do opisania go wystarczy tylko obrot od 0 do pi)
    if rotZ < 0:
        beta = math.pi + rotZ
    else:
        beta = rotZ

    # Wyznaczanie kata ustawienia stolu wzgledem ukladu bazy
    if T_B_Table.p.x() >= 0:
        alpha = math.atan(T_B_Table.p.y() / (T_B_Table.p.x() + 0.01))
    else:
        if T_B_Table.p.y() >= 0:
            alpha = math.pi + math.atan(T_B_Table.p.y() / T_B_Table.p.x())
        else:
            alpha = -math.pi + math.atan(T_B_Table.p.y() / T_B_Table.p.x())

    # Wybor punktow odkladania puszki w oparciu o dopasowanie do wymiarow stolu
    gamma = alpha - beta
    print "Gamma: ", gamma
    const = math.atan(0.9/1.1) # kat wynikajacy ze stosuneku wymiarow stolu
    if (gamma <= const and gamma >= -const) or gamma <= -2*math.pi + const or gamma >= math.pi - const or (gamma <= -math.pi + const and gamma >=-math.pi - const):
        z = math.fabs(1.1/math.cos(math.fabs(gamma)))
    elif (gamma >= const and gamma <= math.pi -const) or (gamma <=-const and gamma >= -math.pi + const) or(gamma <= -math.pi-const and gamma >= -2*math.pi+ const):
        z = math.fabs(0.9/math.sin(math.fabs(gamma)))
 
    print "Z: ", z

    # Wyznaczanie kata obrotu korpusu
    if alpha >= math.pi:
        q_map_1['torso_0_joint'] = 1.56;
    elif alpha <= -math.pi:
        q_map_1['torso_0_joint'] = -1.56;
    else:
        q_map_1['torso_0_joint'] = alpha;
    print "kat: ", alpha

    posX = T_B_Table.p.x() - 0.25*math.cos(alpha) - z*math.cos(alpha) 
    posY = T_B_Table.p.y() - 0.25*math.sin(alpha) - z*math.sin(alpha)

    print "posX: ", posX, "posY: ", posY


    exitError(0)

