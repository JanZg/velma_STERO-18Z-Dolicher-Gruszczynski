#!/usr/bin/env python2

## Runs test for grippers motions.
# @ingroup integration_tests
# @file test_grippers.py
# @namespace scripts.test_grippers Integration test

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
from velma_common import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":

    rospy.init_node('grippers_test', anonymous=False)

    rospy.sleep(1)

    velma = VelmaInterface()
    print "waiting for init..."

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    if velma.enableMotors() != 0:
        exitError(14)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    print "reset left"
    velma.resetHandLeft()
    if velma.waitForHandLeft() != 0:
        exitError(2)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
        exitError(3)

    print "reset right"
    velma.resetHandRight()
    if velma.waitForHandRight() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), [0,0,0,0]):
        exitError(5)

    dest_q = [90.0/180.0*math.pi,0,0,0]
    print "move left:", dest_q
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
        print velma.getHandLeftCurrentConfiguration(), dest_q
        exitError(7)

    dest_q = [90.0/180.0*math.pi,0,0,0]
    print "move right:", dest_q
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(9)

    dest_q = [0,90.0/180.0*math.pi,0,180.0/180.0*math.pi]
    print "move right:", dest_q
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)

    print "reset left"
    velma.resetHandLeft()
    if velma.waitForHandLeft() != 0:
        exitError(12)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
        exitError(13)

    print "reset right"
    velma.resetHandRight()
    if velma.waitForHandRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), [0,0,0,0]):
        exitError(15)

    exitError(0)

