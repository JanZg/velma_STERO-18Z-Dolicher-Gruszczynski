#!/usr/bin/env python2

# Plik odpowiadajacy za przenoszenie obiektu z jednego stolu na drugi.

import roslib

roslib.load_manifest('velma_task_cs_ros_interface')

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

def moveCloserToDoor(q_dest):

    velma.moveJoint(q_dest, 3, start_time=0.01, position_tol=15.0/180.0*math.pi)
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


def moveWristToPos(T_B_Trd, move_time=3, tol=PyKDL.Twist(PyKDL.Vector(0.05, 0.05, 0.05), PyKDL.Vector(0.05, 0.05, 0.05))):
    """!
    Ruch nadgarstka do zadanej pozycji.

    @param T_B_Trd      PyKDL.Frame: Pozycja nadgarstka robota wzgledem ukladu wspolrzednych bazy.
    """
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    if not velma.moveCartImpRight([T_B_Trd], [move_time], None, None, None, None,
                                  PyKDL.Wrench(PyKDL.Vector(0.001, 0.001, 0.001), PyKDL.Vector(0.001, 0.001, 0.001)),
                                  start_time=0.01,
                                  path_tol=tol,
                                  ):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        if not velma.moveCartImpRightCurrentPos(start_time=0.01):
            exitError(98)
        return 0
    else:
        return 1


def checkIfInContact(dest_conf):
    """!
    Fukcja sprawdza czy chwycony obiekt nie zostal opuszczony

    @param dest_conf      tablcia: Tablica przechowujaca zadane pozycje kazdego z palcow chwytaka

    """
    if not isHandConfigurationClose(velma.getHandRightCurrentConfiguration(), dest_conf, tolerance=0.05):
        print "OK!"
    else:
        print "HALO?! GDZIE JEST SZAFKA?!?!"
        moveCloserToDoor(q_map_1)
        moveCloserToDoor(q_map_starting)
        exitError(15)

def prepareGripper(dest_q):
    """!
    Przygotowywanie palcow do chwytu.
    """
    velma.moveHandRight(dest_q, [4, 4, 4, 4], [2000, 2000, 2000, 2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)


def calculatePosition(T_B_Cabinet, x, y, z):
    """!
    Funkcja wyznacza pozycje do ktorej ma zostac wykonany ruch w celu otwarcia drzwi szafki.

    @param T_B_Table    PyKDL.Frame: Pozycja szafki.
    @param x            float: Wspolrzedna x punktu w ukladzie wspolrzednych szafki
    @param y            float: Wspolrzedna y punktu w ukladzie wspolrzednych szafki
    @param z            float: Wysokosc szafki

    @return Zwraca pozycje docelowa chwytaka w ukladzie wspolrzednych bazy (parametry posX, posY, posZ)
    """
    (rotX, rotY, rotZ) = T_B_Cabinet.M.GetRPY()

    posX = T_B_Cabinet.p.x() + math.cos(rotZ)*x - math.sin(rotZ)*y
    posY = T_B_Cabinet.p.y() + math.sin(rotZ)*x + math.cos(rotZ)*y
    posZ = T_B_Cabinet.p.z() + z/2.0 - 0.23
    alpha = - math.pi + rotZ

    if alpha < -math.pi:
        alpha = 2*math.pi + alpha

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
    width = 0.6; length = 0.3; height = 0.7

    # Wyznaczanie kata obrotu korpusu oraz docelowego obrotu nadgarstka
    if T_B_Cabinet.p.x() > 0:   # Przestrzen przed robotem
        q_map_1['torso_0_joint'] = math.atan(T_B_Cabinet.p.y() / T_B_Cabinet.p.x()) # Obrot korpusu
    else:                    # Przestrzen za robotem
        q_map_1['torso_0_joint'] = math.copysign(1.56, T_B_Cabinet.p.y())

    print "Ustawianie sie w kierunku szafki"
    moveCloserToDoor(q_map_1)

    print "Przygotowywanie chwytu"
    q = [80.0 / 180.0 * math.pi, 80.0 / 180.0 * math.pi, 80.0 / 180.0 * math.pi, 180.0 / 180.0 * math.pi]
    prepareGripper(q)

    print "Ruch w strone prawych drzwi"
    (posX, posY, posZ, alpha) = calculatePosition(T_B_Cabinet, 0.50, 0.15, 0.7)
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, alpha), PyKDL.Vector(posX, posY, posZ))
    moveWristToPos(frame, move_time=2)

    print "Wykrywanie drzwi szafki"
    (posX, posY, posZ, alpha) = calculatePosition(T_B_Cabinet, 0.3, 0.15, 0.7)
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, alpha), PyKDL.Vector(posX, posY, posZ))
    if (moveWristToPos(frame, move_time=4)):
        print "HALO?! GDZIE JEST SZAFKA?!?!"
        moveCloserToDoor(q_map_1)
        moveCloserToDoor(q_map_starting)
    else:
        print "OK!"

    print "Wyjscie z kolizji"
    Wrist_pos = velma.getTf("B", "Wr")
    # Wyznaczanie pozycji nadgarstka w ukladzie wspolrzednych szafki
    collX = -(Wrist_pos.p.x()- T_B_Cabinet.p.x())*math.cos(alpha) - (Wrist_pos.p.y()-T_B_Cabinet.p.y())*math.sin(alpha)
    collY =  (Wrist_pos.p.x()- T_B_Cabinet.p.x()) * math.sin(alpha) - (Wrist_pos.p.y()-T_B_Cabinet.p.y()) * math.cos(alpha)
    print posX, posY
    (posX, posY, posZ, alpha) = calculatePosition(T_B_Cabinet, collX + 0.02, collY, 0.7)
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, alpha), PyKDL.Vector(posX, posY, posZ))
    moveWristToPos(frame, move_time=3, tol=PyKDL.Twist(PyKDL.Vector(0.2, 0.2, 0.2), PyKDL.Vector(0.2, 0.2, 0.2)))


    #q = [math.pi/2.0, math.pi/2.0, math.pi/2.0, math.pi]
    #prepareGripper(q)

    print "Ruch w kierunku uchwytu"
    (posX, posY, posZ, alpha) = calculatePosition(T_B_Cabinet, collX + 0.02, collY - 0.25, 0.7)
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, alpha), PyKDL.Vector(posX, posY, posZ))
    if (moveWristToPos(frame, move_time=4)):
        print "HALO?! GDZIE JEST UCHWYT?!?!"
        moveCloserToDoor(q_map_1)
        moveCloserToDoor(q_map_starting)
    else:
        print "OK!"

    print "Wyrownanie do uchwytu"
    (posX, posY, posZ, alpha) = calculatePosition(T_B_Cabinet, collX + 0.02, 0.0, 0.7)
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, alpha), PyKDL.Vector(posX, posY, posZ))
    moveWristToPos(frame, move_time=2, tol=PyKDL.Twist(PyKDL.Vector(0.2, 0.2, 0.2), PyKDL.Vector(0.2, 0.2, 0.2)))

    print "Otwieranie i aproksymacja promienia drzwi szadki"


    exitError(0)
