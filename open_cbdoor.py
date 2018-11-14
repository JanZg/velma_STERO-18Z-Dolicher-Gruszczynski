#!/usr/bin/env python2

# Plik odpowiadajacy za przenoszenie obiektu z jednego stolu na drugi.

import roslib

roslib.load_manifest('velma_task_cs_ros_interface')

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

def moveCloserToDoor(q_dest):

    velma.moveJoint(q_dest, 3.0, start_time=0.1, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
    print "Checking if the configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_dest, js[1], tolerance=0.1):
        exitError(10)

def prepareGrippersForMove():
    """!
    Fukcja przygotowuje pralce robota poprzez ich skladanie, w celu ulatwienia zaplanowania trasy
    oraz wykonania samego ruchu.
    """
    dest_q = [90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, 0]
    velma.moveHandRight(dest_q, [3, 3, 3, 3], [2000, 2000, 2000, 2000], 1000, hold=True)
    velma.moveHandLeft(dest_q, [3, 3, 3, 3], [2000, 2000, 2000, 2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    if velma.waitForHandLeft() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose(velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)
    if not isHandConfigurationClose(velma.getHandLeftCurrentConfiguration(), dest_q):
        exitError(11)


def switchToCartImp():
    """!
    Fukcja przelacza robota do trybu cart_imp.

    """
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)


def moveWristToPos(T_B_Trd):
    """!
    Ruch nadgarstka do zadanej pozycji.

    @param T_B_Trd      PyKDL.Frame: Pozycja nadgarstka robota wzgledem ukladu wspolrzednych bazy.
    """
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    if not velma.moveCartImpRight([T_B_Trd], [1.5], None, None, None, None,
                                  PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.01):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)


def prepareGripper():
    """!
    Przygotowywanie palcow do chwytu.
    """
    dest_q = [80.0 / 180.0 * math.pi, 80.0 / 180.0 * math.pi, 80.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi]
    velma.moveHandRight(dest_q, [2, 2, 2, 2], [2000, 2000, 2000, 2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)


def calculatePosition(T_B_Cabinet, x, y, z):
    """!
    Funkcja wyznacza pozycje do ktorej ma zostac wykonany ruch w celu otwarcia drzwi szafki.

    @param T_B_Table    PyKDL.Frame: Pozycja szafki.
    @param x            float: Szerokosc szafki
    @param y            float: Dlugosc szafki
    @param z            float: Wysokosc szafki

    @return Zwraca pozycje docelowa chwytaka w ukladzie wspolrzednych bazy (parametry posX, posY, posZ)
    """
    (rotX, rotY, rotZ) = T_B_Cabinet.M.GetRPY()

    posX = T_B_Cabinet.p.x() + math.cos(rotZ)*(x/2.0 + 0.2) - math.sin(rotZ)*0.15
    posY = T_B_Cabinet.p.y() + math.sin(rotZ)*(x/2.0 + 0.2) + math.cos(rotZ)*0.15
    posZ = T_B_Cabinet.p.z() + z/2.0
    alpha = rotZ
    return [posX, posY, posZ, alpha]


if __name__ == "__main__":

    # Definicje pozycji poczatkowej i konfiguracji prawej reki do chwytu
    q_map_starting = {'torso_0_joint': 0,
                      'right_arm_0_joint': -0.3, 'left_arm_0_joint': 0.3,
                      'right_arm_1_joint': -1.8, 'left_arm_1_joint': 1.8,
                      'right_arm_2_joint': 1.25, 'left_arm_2_joint': -1.25,
                      'right_arm_3_joint': 0.85, 'left_arm_3_joint': -0.85,
                      'right_arm_4_joint': 0, 'left_arm_4_joint': 0,
                      'right_arm_5_joint': -0.5, 'left_arm_5_joint': 0.5,
                      'right_arm_6_joint': 0, 'left_arm_6_joint': 0}

    q_map_1 = {'torso_0_joint': 0,
               'right_arm_0_joint': 0.282, 'left_arm_0_joint': 0.3,
               'right_arm_1_joint': -1.814, 'left_arm_1_joint': 1.8,
               'right_arm_2_joint': 1.184, 'left_arm_2_joint': -1.25,
               'right_arm_3_joint': 2.00, 'left_arm_3_joint': -0.85,
               'right_arm_4_joint': 0.294, 'left_arm_4_joint': 0,
               'right_arm_5_joint': -1.211, 'left_arm_5_joint': 0.5,
               'right_arm_6_joint': -0.51, 'left_arm_6_joint': 0}

    rospy.init_node('test_cimp_pose')
    rospy.sleep(0.5)

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

    # --------------------------------- END OF INIT -----------------------------------------

    print "Pobieranie pozycji szafki"
    T_B_Cabinet = velma.getTf("B", "cabinet_door")

    print "Wyznaczanie pozycji docelowej"
    (posX, posY, posZ, alpha) = calculatePosition(T_B_Cabinet, 0.6, 0.3, 0.7)

    print "Ruch w poblize szafki"
    moveCloserToDoor(q_map_1)

    print "Przygotowywanie chwytu"
    prepareGripper()



    exitError(0)
