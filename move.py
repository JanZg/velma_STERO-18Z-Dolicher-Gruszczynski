#!/usr/bin/env python2

# Plik odpowiadajacy za przenoszenie piwa z jednego stolu na drugi. 

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    # define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_1 = {'torso_0_joint':0,
        'right_arm_0_joint':-0.45,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.98,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.52,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':2.09,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':-0.03,      'left_arm_4_joint':0,
        'right_arm_5_joint':-2.09,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    rospy.init_node('test_cimp_pose')

    rospy.sleep(0.5)

    print "This test/tutorial executes simple motions"\
        " in Cartesian impedance mode.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    print "Reading octomap to planner"
    p = Planner(velma.maxJointTrajLen())
    p.waitForInit()
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)
    # planning...

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)


    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(20):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print "The trajectory could not be completed, retrying..."
                continue
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            exitError(6)


    if velma.enableMotors() != 0:
        exitError(14)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(3)

    planAndExecute(q_map_starting)

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(10)

    # get initial configuration
    js_init = velma.getLastJointState()

#    planAndExecute(q_map_1)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)

    print "Reset tools for both arms..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_B_Wl = velma.getTf("B", "Wl")
    if not velma.moveCartImpRight([T_B_Wr], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if not velma.moveCartImpLeft([T_B_Wl], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)
#----------------------------------------------------------------------------------------------
    T_B_Beer = velma.getTf("B", "beer")
    planAndExecute(q_map_starting)
	
    # cwiartka 1
    if T_B_Beer.p.x() >= 0 and T_B_Beer.p.y() >= 0:
        angle = 1.56
    # cwiartka 2
    elif T_B_Beer.p.x() < 0 and T_B_Beer.p.y() >= 0:
        angle = -1.56
    # cwiartka 3
    elif T_B_Beer.p.x() < 0 and T_B_Beer.p.y() < 0:
        angle = math.atan(T_B_Beer.p.x()/abs(T_B_Beer.p.y()))
    # cwiartka 4
    elif T_B_Beer.p.x() >= 0 and T_B_Beer.p.y() < 0:
        angle = math.atan(T_B_Beer.p.x()/abs(T_B_Beer.p.y()))

    
    q_map_starting['torso_0_joint'] = angle; # slownik ustawimay torso na 0    

    planAndExecute(q_map_1)
    rospy.sleep(5)
    planAndExecute(q_map_starting)

    exitError(0)

