#!/usr/bin/env python2

# Plik odpowiadajacy za przenoszenie obiektu z jednego stolu na drugi.

import roslib

roslib.load_manifest('velma_task_cs_ros_interface')
from threading import Thread

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import MarkerPublisher, exitError
from functions import *

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm

class MarkerPublisherThread:
    """!
    Klasa wykorzystywana do wizualizacji przenoszonego obiektu.
    """
    def threaded_function(self, obj):
        pub = MarkerPublisher("attached_objects")
        while not self.stop_thread:
            pub.publishSinglePointMarker(PyKDL.Vector(), 1, r=1, g=0, b=0, a=1, namespace='default',
                                         frame_id=obj.link_name, m_type=Marker.CYLINDER,
                                         scale=Vector3(0.06, 0.06, 0.23), T=pm.fromMsg(obj.object.primitive_poses[0]))
            try:
                rospy.sleep(0.1)
            except:
                break

        try:
            pub.eraseMarkers(0, 10, namespace='default')
            rospy.sleep(0.5)
        except:
            pass

    def __init__(self, obj):
        self.thread = Thread(target=self.threaded_function, args=(obj,))

    def start(self):
        self.stop_thread = False
        self.thread.start()

    def stop(self):
        self.stop_thread = True
        self.thread.join()


# Planowanie oraz wykonuje ruch do zadanej pozycji stawow
# q_dest - slownik zawierajacy pozycje zadane poszczegolnych stawow
def planAndExecute(q_dest):
    """!
    Fukcja planuje oraz wykonuje ruch do zadanej pozycji stawow

    @param q_dest        slownik: Slownik  {nazwa_stawu:pozycja_docelowa} zawierajacy docelowe pozycje stawow.
    """
    print "Przygotowanie chwytakow do ruchu"
    dest_q = [90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, 90.0 / 180.0 * math.pi, 0]
    velma.moveHandRight(dest_q, [2, 2, 2, 2], [2000, 2000, 2000, 2000], 1000, hold=True)
    velma.moveHandLeft(dest_q, [2, 2, 2, 2], [2000, 2000, 2000, 2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    if velma.waitForHandLeft() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose(velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)
    if not isHandConfigurationClose(velma.getHandLeftCurrentConfiguration(), dest_q):
        exitError(11)

    goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(20):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.4,
                      max_acceleration_scaling_factor=0.4, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0*math.pi):
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


def planAndExecuteAttached(q_map):
    """!
    Fukcja planuje oraz wykonuje ruch do zadanej pozycji stawow z uwzglednieniem przenoszonego obiektu

    @param q_dest        slownik: Slownik  {nazwa_stawu:pozycja_docelowa} zawierajacy docelowe pozycje stawow.
    """


    print "Creating a virtual object attached to gripper..."

    # for more details refer to ROS docs for moveit_msgs/AttachedCollisionObject
    object1 = AttachedCollisionObject()
    object1.link_name = "right_HandGripLink"
    object1.object.header.frame_id = "right_HandGripLink"
    object1.object.id = "object1"
    object1_prim = SolidPrimitive()
    object1_prim.type = SolidPrimitive.CYLINDER
    object1_prim.dimensions = [None, None]  # set initial size of the list to 2
    object1_prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = 0.23
    object1_prim.dimensions[SolidPrimitive.CYLINDER_RADIUS] = 0.06
    object1_pose = pm.toMsg(PyKDL.Frame(PyKDL.Rotation.RotY(math.pi / 2)))
    object1.object.primitives.append(object1_prim)
    object1.object.primitive_poses.append(object1_pose)
    object1.object.operation = CollisionObject.ADD
    object1.touch_links = ['right_HandPalmLink',
                           'right_HandFingerOneKnuckleOneLink',
                           'right_HandFingerOneKnuckleTwoLink',
                           'right_HandFingerOneKnuckleThreeLink',
                           'right_HandFingerTwoKnuckleOneLink',
                           'right_HandFingerTwoKnuckleTwoLink',
                           'right_HandFingerTwoKnuckleThreeLink',
                           'right_HandFingerThreeKnuckleTwoLink',
                           'right_HandFingerThreeKnuckleThreeLink']

    # print "Publishing the attached object marker on topic /attached_objects"
    # pub = MarkerPublisherThread(object1)
    # pub.start()

    print "Moving to valid position, using planned trajectory."
    goal_constraint_1 = qMapToConstraints(q_map, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(20):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.15,
                      max_acceleration_scaling_factor=0.2, planner_id="RRTConnect", attached_collision_objects=[object1])
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=20.0/180.0 * math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map, js[1]):
        print "Nie udalo sie zaplanowac ruchu do pozycji docelowej, puszka nie zostala odlozona."
        exitError(6)

# Przelaczanie robota do trybu cart_imp
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


# Ruch nadgarstka do zadanej pozycji, T_B_Trd = PyKDL.Frame (przeksztalcenie wzgledem bazy robota)
def moveWristToPos(T_B_Trd):
    """!
    Ruch nadgarstka do zadanej pozycji.

    @param T_B_Trd      PyKDL.Frame: Pozycja nadgarstka robota wzgledem ukladu wspolrzednych bazy.
    """
    if not velma.moveCartImpRight([T_B_Trd], [1.5], None, None, None, None,
                                  PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)


def grabObject():
    """!
    Chwytanie obiektu.
    """
    dest_q = [80.0 / 180.0 * math.pi, 80.0 / 180.0 * math.pi, 80.0 / 180.0 * math.pi, 0]
    velma.moveHandRight(dest_q, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)

    if not isHandConfigurationClose(velma.getHandRightCurrentConfiguration(), dest_q):
        print "Puszka chwycona"
    else:
        print "Nie udalo sie chwycic puszki. Powrot do pozycji poczatkowej"
        planAndExecute(q_map_starting)
        exitError(15)


def checkIfStillGrabbed(dest_conf):
    """!
    Fukcja sprawdza czy chwycony obiekt nie zostal opuszczony

    @param dest_conf      tablcia: Tablica przechowujaca zadane pozycje kazdego z palcow chwytaka

    """
    if isHandConfigurationClose(velma.getHandRightCurrentConfiguration(), dest_conf):
        print "Niepowodzenie - puszka zostala opuszczona. Powrot do pozycji poczatkowej."
        planAndExecute(q_map_starting)
        exitError(15)

# Wyznaczanie pozycji do ktorej ma zostac wykonany ruch w celu odlozenia puszki na drugim stole
# oraz wyznaczanie kata obrotu korpusu tak by byl on ustawiony przodem do drugiego stolu
# T_B_Table - pozycja stolu
# x, y - wymiary stolu
def calculatePosition(T_B_Table, x, y, z):
    """!
    Funkcja wyznacza pozycje do ktorej ma zostac wykonany ruch w celu odlozenia puszki na drugim stole.
    Wyznaczany jest takze kat obrotu korpusu tak, by robot byl ustawiony przodem do drugiego stolu.

    @param T_B_Table    PyKDL.Frame: Pozycja drugiego stolu wzgledem ukladu wspolrzednych bazy.
    @param x            float: Dlugosc stolu/2
    @param y            float: Szerokosc stolu/2
    @param z            float: Wysokosc stolu

    @return Zwraca pozycje docelowa nadgarstka w ukladzie wspolrzednych bazy (parametry posX, posY, posZ)
            oraz kat obrotu korpusu (parametr alpha)
    """
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
    const = math.atan(y / x)  # kat wynikajacy ze stosuneku wymiarow stolu
    if (gamma <= const and gamma >= -const) or gamma <= -2 * math.pi + const or gamma >= math.pi - const or (
            gamma <= -math.pi + const and gamma >= -math.pi - const):
        c = math.fabs(x / math.cos(math.fabs(gamma)))
    elif (gamma >= const and gamma <= math.pi - const) or (gamma <= -const and gamma >= -math.pi + const) or (
            gamma <= -math.pi - const and gamma >= -2 * math.pi + const):
        c = math.fabs(y / math.sin(math.fabs(gamma)))

    # Wyznaczanie kata obrotu korpusu
    if alpha >= 1.56:
        q_map_1['torso_0_joint'] = 1.56
    elif alpha <= -1.56:
        q_map_1['torso_0_joint'] = -1.56
    else:
        q_map_1['torso_0_joint'] = alpha

    posX = T_B_Table.p.x() - (0.27 + c) * math.cos(alpha)
    posY = T_B_Table.p.y() - (0.27 + c) * math.sin(alpha)
    posZ = z + 0.25
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

    print "Reading octomap to planner"
    p = Planner(velma.maxJointTrajLen())
    p.waitForInit()
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)

    print "Planning motion to the starting position using set of all joints..."
    planAndExecute(q_map_starting)


    print "Pobieranie pozycji puszki oraz wyznaczanie kata obrotu"
    T_B_Beer = velma.getTf("B", "beer")

    # Wyznaczanie kata obrotu korpusu oraz docelowego obrotu nadgarstka
    if T_B_Beer.p.x() > 0:   # Przestrzen przed robotem
        angle = math.atan(T_B_Beer.p.y() / T_B_Beer.p.x()) # Obrot korpusu
        alpha = angle                                      # Obrot nadgarstka
    else:                    # Przestrzen za robotem
        angle = math.copysign(1.56, T_B_Beer.p.y())
        alpha = math.copysign(math.pi, T_B_Beer.p.y()) + math.atan(T_B_Beer.p.y() / T_B_Beer.p.x())


    q_map_1['torso_0_joint'] = angle  # slownik ustawimay torso na 0

    getch = Getch()
    print "Pess any key to continue..."
    getch()

    print "Planning motion to the goal position using set of all joints..."
    planAndExecute(q_map_1)  # Ustawianie sie przodem do puszki z jednoczesnym podnoszeniem prawej reki nad stoly

    print "Pess any key to continue..."
    getch()

    print "Przygotowanie prawej reki do chwytu"
    dest_q = [0, 0, 0, 0]
    velma.moveHandRight(dest_q, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose(velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)


    print "Switch to cart_imp mode..."
    switchToCartImp()

    # Wyliczanie pozycji nadgarstka (musi byc on nieco oddalony od obiektu chwytanego)
    print "Wyznaczanie pozycji docelowej chwytaka"
    posX = T_B_Beer.p.x() - 0.275 * math.cos(alpha)
    posY = T_B_Beer.p.y() - 0.275 * math.sin(alpha)
    posZ = T_B_Beer.p.z() + 0.12

    print "Pess any key to continue..."
    getch()

    print "Ruch do puszki..."
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, angle), PyKDL.Vector(posX, posY, posZ))
    moveWristToPos(frame)

    print "Pess any key to continue..."
    getch()

    print "Chwytanie puszki..."
    grabObject()

    print "Pess any key to continue..."
    getch()

    print "Podnoszenie puszki..."
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, angle),
                        PyKDL.Vector(posX - 0.15 * math.cos(angle), posY - 0.15 * math.sin(angle), posZ + 0.2))
    moveWristToPos(frame)
    checkIfStillGrabbed(dest_q)


    print "Pobieranie pozycji drugiego stolu"
    Table1 = velma.getTf("B", "table_1")
    Table2 = velma.getTf("B", "table_2")
    # Wybor stolu na ktory puszka ma zostac przeniesiona
    if math.sqrt(math.pow(T_B_Beer.p.x() - Table1.p.x(), 2) + math.pow(T_B_Beer.p.y() - Table1.p.y(), 2)) > math.sqrt(
            math.pow(T_B_Beer.p.x() - Table2.p.x(), 2) + math.pow(T_B_Beer.p.y() - Table2.p.y(), 2)):
        New_Table = Table1
        Table_x = 0.60  # Wymiary stolu
        Table_y = 0.25
        Table_z = 1.05
    else:
        New_Table = Table2
        Table_x = 1.15  # Wymiary stolu
        Table_y = 0.95
        Table_z = 1

    print "Wyznaczanie pozycji docelowej nadgarstka oraz kata obrotu bazy)"
    (posX, posY, posZ, alpha) = calculatePosition(New_Table, Table_x, Table_y, Table_z)

    print "Pess any key to continue..."
    getch()

    print "Planning motion to the goal position using set of all joints (with object attached)..."
    planAndExecuteAttached(q_map_1)
    checkIfStillGrabbed(dest_q)


    print "Switch to cart_imp mode..."
    switchToCartImp()

    print "Pess any key to continue..."
    getch()

    print "Ruch nadgarstka do miejsca w ktorym puszka ma zostac odlozona"
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, alpha), PyKDL.Vector(posX, posY, posZ))
    moveWristToPos(frame)

    print "Pess any key to continue..."
    getch()

    print "Opuszczanie puszki"
    dest_q = [0, 0, 0, 0]
    velma.moveHandRight(dest_q, [3, 3, 3, 3], [2000, 2000, 2000, 2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)

    print "Pess any key to continue..."
    getch()

    print "Wycofywanie reki"
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, alpha),
                        PyKDL.Vector(posX - 0.15 * math.cos(alpha), posY - 0.15 * math.sin(alpha), posZ + 0.24))
    moveWristToPos(frame)

    print "Pess any key to continue..."
    getch()

    print "Powrot do pozycji poczatkowej"
    planAndExecute(q_map_starting)

    exitError(0)
