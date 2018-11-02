#!/usr/bin/env python2

# Plik odpowiadajacy za przenoszenie piwa z jednego stolu na drugi. 

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError



if __name__ == "__main__":

    # Definicje pozycji poczatkowej i konfiguracji prawej reki do chwytu
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_1 = {'torso_0_joint':0,
        'right_arm_0_joint': 0.282,   'left_arm_0_joint':0.3,
        'right_arm_1_joint': -1.814,   'left_arm_1_joint':1.8,
        'right_arm_2_joint': 1.184,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint': 2.00,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint': 0.0,      'left_arm_4_joint':0,
        'right_arm_5_joint': -1.669,   'left_arm_5_joint':0.5,
        'right_arm_6_joint': -0.51,      'left_arm_6_joint':0 }

    rospy.init_node('test_cimp_pose')
    rospy.sleep(0.5)

    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
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
        rospy.sleep(0.5)
        print "calculating difference between desiread and reached pose..."
        T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
        print T_B_T_diff
        if T_B_T_diff.vel.Norm() > 0.1 or T_B_T_diff.rot.Norm() > 0.1:
            exitError(10)

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"


    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(14)

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

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Reading octomap to planner"
    p = Planner(velma.maxJointTrajLen())
    p.waitForInit()
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)
    # planning...

    #print "Planning motion to the starting position using set of all joints..."   
    #planAndExecute(q_map_starting)

    print "Switch to cart_imp mode..."
    switchToCartImp()

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

    print "Pobieranie pozycji puszki oraz wyznaczanie kata obrotu"
    T_B_Beer = velma.getTf("B", "beer")
	
    # cwiartka 1
    if T_B_Beer.p.x() > 0 and T_B_Beer.p.y() >= 0:
        angle = math.atan(T_B_Beer.p.y()/T_B_Beer.p.x())
    # cwiartka 2
    elif T_B_Beer.p.x() <= 0 and T_B_Beer.p.y() >= 0:
        angle = 1.56
    # cwiartka 3
    elif T_B_Beer.p.x() <= 0 and T_B_Beer.p.y() < 0:
        angle = -1.56
    # cwiartka 4
    elif T_B_Beer.p.x() > 0 and T_B_Beer.p.y() < 0:
        angle = math.atan(T_B_Beer.p.y()/T_B_Beer.p.x())

    q_map_1['torso_0_joint'] = angle; # slownik ustawimay torso na 0    

    print "Planning motion to the goal position using set of all joints..."
    #planAndExecute(q_map_1)  # Ustawianie sie przodem do puszki z jednoczesnym podnoszeniem prawej reki nad stoly


    print "Switch to cart_imp mode..."
    switchToCartImp()

    # Wyliczanie pozycji nadgarstka (nadgarstek bedzie ustawiony rownolegle do prostej przechodzacej przez srodki korpusu i puszki)
    # Musi byc on takze nieco oddalony od puszki (funckja do poruszania reka przyjmuje pozycje nadgarstka a nie samej koncowki)
    print "Wyznaczanie pozycji docelowej chwytaka"
    if T_B_Beer.p.x() >= 0:  # W tym przypadku kat do ktorego ma sie obrocic nadgarstek jest taki sam jak poczatkowy kat obrotu korpusu
        x = T_B_Beer.p.x() - 0.29*math.cos(angle)
        y = T_B_Beer.p.y() - 0.29*math.sin(angle)
    else:  # Przestrzen za robotem
        angle = math.atan(T_B_Beer.p.y()/T_B_Beer.p.x())  # Wyliczanie kata do obliczenia wartosci funkcji sin i cos
        x = T_B_Beer.p.x() + 0.29*math.cos(angle)
        y = T_B_Beer.p.y() + 0.29*math.sin(angle)
        if T_B_Beer.p.y() >=0:    # Wyliczanie kata do jakiego ma sie obrocic nadgartek robot (jak ma byc obrocony wzgledem bazy)
            angle = 90.0/180.0*math.pi + angle
        else:
            angle = -90.0/180.0*math.pi + angle

    print "Ruch do puszki..."
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, angle), PyKDL.Vector(x, y, T_B_Beer.p.z()+0.1))
    moveWristToPos(frame)

    print "Chwytanie puszki..."
    dest_q = [90.0/180.0*math.pi,90.0/180.0*math.pi,90.0/180.0*math.pi,0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        print "Puszka chwycona"

    print "Podnoszenie puszki..."
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, angle), PyKDL.Vector(x, y, T_B_Beer.p.z()+0.1+0.1))
    moveWristToPos(frame)

    exitError(0)

