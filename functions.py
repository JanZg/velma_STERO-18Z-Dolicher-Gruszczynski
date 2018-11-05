#!/usr/bin/env python2

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
import PyKDL
from threading import Thread

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import MarkerPublisher, exitError

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm

# define a function for frequently used routine in this test
def planAndExecute(q_dest):
    print "Przygotowanie chwytakow do ruchu"
    dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    if velma.waitForHandLeft() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
        exitError(11)

    goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(20):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.10, planner_id="RRTConnect")
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

# Przelaczanie robota do trybu cart_imp
def switchToCartImp():
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)

# Ruch nadgarstka do zadanej pozycji, T_B_Trd = PyKDL.Frame (przeksztalcenie wzgledem bazy robota)
def moveWristToPos(T_B_Trd):
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

def calculatePosition(T_B_Table, x, y):
    (rotX, rotY, rotZ) = T_B_Table.M.GetRPY()
    # Wyznaczanie kata obrotu stolu (stol jes symetryczny wiec do opisania go wystarczy tylko obrot od 0 do pi)
    if rotZ < 0:
        beta = math.pi + rotZ
    else:
        beta = rotZ

    # Wyznaczanie kata ustawienia stolu wzgledem ukladu bazy
    if T_B_Table.p.x() >= 0:
        alpha = math.atan(T_B_Table.p.y() / (T_B_Table.p.x() + 0.001))
    else:
        if T_B_Table.p.y() >= 0:
            alpha = math.pi + math.atan(T_B_Table.p.y() / T_B_Table.p.x())
        else:
            alpha = -math.pi + math.atan(T_B_Table.p.y() / T_B_Table.p.x())

    # Wybor punktow odkladania puszki w oparciu o dopasowanie do wymiarow stolu
    gamma = alpha - beta
    const = math.atan(y/x) # kat wynikajacy ze stosuneku wymiarow stolu
    if (gamma <= const and gamma >= -const) or gamma <= -2*math.pi + const or gamma >= math.pi - const or (gamma <= -math.pi + const and gamma >=-math.pi - const):
        z = math.fabs(x/math.cos(math.fabs(gamma)))
    elif (gamma >= const and gamma <= math.pi -const) or (gamma <=-const and gamma >= -math.pi + const) or(gamma <= -math.pi-const and gamma >= -2*math.pi+ const):
        z = math.fabs(y/math.sin(math.fabs(gamma)))


    # Wyznaczanie kata obrotu korpusu
    if alpha >= 1.56:
        q_map_1['torso_0_joint'] = 1.56;
    elif alpha <= -1.56:
        q_map_1['torso_0_joint'] = -1.56;
    else:
        q_map_1['torso_0_joint'] = alpha;

    posX = T_B_Table.p.x() - 0.27*math.cos(alpha) - z*math.cos(alpha) 
    posY = T_B_Table.p.y() - 0.27*math.sin(alpha) - z*math.sin(alpha)
    posZ = T_B_Table.p.z() + 0.1
    return [posX, posY, posZ, alpha]
