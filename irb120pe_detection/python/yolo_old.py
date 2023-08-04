#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# yolo.py

# Required to include ROS2 and its components:
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import ROS2 MSG, SRV and ACTION (s):
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from irb120pe_data.action import Robmove
from ros2srrc_data.action import Move
from ros2srrc_data.msg import Action
from abb_robot_msgs.srv import SetIOSignal

# Required to use OpenCV:
import cv2
from cv2 import aruco
import numpy as np

# Extra-required libraries:
import os
import time
import math

# Data classes:
from dataclasses import dataclass

# Required to include YOLOv8 module:
from ultralytics import YOLO

# Define GLOBAL VARIABLE -> RES:
@dataclass
class feedback:
    MESSAGE: String
    SUCCESS: bool
RES = feedback("null", False)
RES_ABB = "null"

# =============================================================================== #
# PRE-DEFINED ROBOT POSES:

PlaceBLUE_app = Pose()
PlaceBLUE_app.position.x = 0.15
PlaceBLUE_app.position.y = 0.145
PlaceBLUE_app.position.z = 1.111
PlaceBLUE_app.orientation.x = 0.707
PlaceBLUE_app.orientation.y = 0.707
PlaceBLUE_app.orientation.z = 0.0
PlaceBLUE_app.orientation.w = 0.0
PlaceBLUE = Pose()
PlaceBLUE.position.x = 0.15
PlaceBLUE.position.y = 0.145
PlaceBLUE.position.z = 1.08
PlaceBLUE.orientation.x = 0.707
PlaceBLUE.orientation.y = 0.707
PlaceBLUE.orientation.z = 0.0
PlaceBLUE.orientation.w = 0.0

PlaceBLACK_app = Pose()
PlaceBLACK_app.position.x = 0.105
PlaceBLACK_app.position.y = 0.145
PlaceBLACK_app.position.z = 1.111
PlaceBLACK_app.orientation.x = 0.707
PlaceBLACK_app.orientation.y = 0.707
PlaceBLACK_app.orientation.z = 0.0
PlaceBLACK_app.orientation.w = 0.0
PlaceBLACK = Pose()
PlaceBLACK.position.x = 0.105
PlaceBLACK.position.y = 0.145
PlaceBLACK.position.z = 1.08
PlaceBLACK.orientation.x = 0.707
PlaceBLACK.orientation.y = 0.707
PlaceBLACK.orientation.z = 0.0
PlaceBLACK.orientation.w = 0.0

PlaceWHITE_app = Pose()
PlaceWHITE_app.position.x = 0.105
PlaceWHITE_app.position.y = 0.100
PlaceWHITE_app.position.z = 1.111
PlaceWHITE_app.orientation.x = 0.707
PlaceWHITE_app.orientation.y = 0.707
PlaceWHITE_app.orientation.z = 0.0
PlaceWHITE_app.orientation.w = 0.0
PlaceWHITE = Pose()
PlaceWHITE.position.x = 0.105
PlaceWHITE.position.y = 0.100
PlaceWHITE.position.z = 1.08
PlaceWHITE.orientation.x = 0.707
PlaceWHITE.orientation.y = 0.707
PlaceWHITE.orientation.z = 0.0
PlaceWHITE.orientation.w = 0.0

PlaceCUBE_app = Pose()
PlaceCUBE_app.position.x = 0.15
PlaceCUBE_app.position.y = 0.100
PlaceCUBE_app.position.z = 1.111
PlaceCUBE_app.orientation.x = 0.707
PlaceCUBE_app.orientation.y = 0.707
PlaceCUBE_app.orientation.z = 0.0
PlaceCUBE_app.orientation.w = 0.0
PlaceCUBE = Pose()
PlaceCUBE.position.x = 0.15
PlaceCUBE.position.y = 0.100
PlaceCUBE.position.z = 1.08
PlaceCUBE.orientation.x = 0.707
PlaceCUBE.orientation.y = 0.707
PlaceCUBE.orientation.z = 0.0
PlaceCUBE.orientation.w = 0.0

# Check FRONTAL FACE:
FacePose_1 = Pose()
FacePose_1.position.x = 0.501
FacePose_1.position.y = 0.525
FacePose_1.position.z = 1.48
FacePose_1.orientation.x = 0.0
FacePose_1.orientation.y = 0.708
FacePose_1.orientation.z = 0.0
FacePose_1.orientation.w = 0.707
# Check BACK FACE:
FacePose_2 = Action()
FacePose_2.action = "MoveR"
FacePose_2.speed = 1.0
FacePose_2.mover.joint = "joint6"
FacePose_2.mover.value = 180.0
FacePose_2_other = Action()
FacePose_2_other.action = "MoveR"
FacePose_2_other.speed = 1.0
FacePose_2_other.mover.joint = "joint6"
FacePose_2_other.mover.value = -180.0
# Check BOTTOM FACE:
FacePose_3 = Pose()
FacePose_3.position.x = 0.63
FacePose_3.position.y = 0.525
FacePose_3.position.z = 1.353
FacePose_3.orientation.x = 0.001
FacePose_3.orientation.y = 0.147
FacePose_3.orientation.z = -0.003
FacePose_3.orientation.w = 0.989
# Check TOP FACE:
FacePose_4 = Pose()
FacePose_4.position.x = 0.525
FacePose_4.position.y = 0.527
FacePose_4.position.z = 1.58
FacePose_4.orientation.x = -0.004
FacePose_4.orientation.y = 0.940
FacePose_4.orientation.z = 0.002
FacePose_4.orientation.w = 0.342

# PLACE OBJECT for DIFFERENT GRASP:
PlaceMidApp = Pose()
PlaceMidApp.position.x = 0.65
PlaceMidApp.position.y = 0.53
PlaceMidApp.position.z = 1.1
PlaceMidApp.orientation.x = 0.0
PlaceMidApp.orientation.y = 1.0
PlaceMidApp.orientation.z = 0.0
PlaceMidApp.orientation.w = 0.0
PlaceMid = Pose()
PlaceMid.position.x = 0.65
PlaceMid.position.y = 0.53
PlaceMid.position.z = 1.07
PlaceMid.orientation.x = 0.0
PlaceMid.orientation.y = 1.0
PlaceMid.orientation.z = 0.0
PlaceMid.orientation.w = 0.0

# Pick cube to CHECK SIDE FACES:
PickSideApp_1 = Action()
PickSideApp_1.action = "MoveR"
PickSideApp_1.speed = 1.0
PickSideApp_1.mover.joint = "joint6"
PickSideApp_1.mover.value = 90.0
PickSideApp_1_other = Action()
PickSideApp_1_other.action = "MoveR"
PickSideApp_1_other.speed = 1.0
PickSideApp_1_other.mover.joint = "joint6"
PickSideApp_1_other.mover.value = -90.0
PickSideApp_2 = Action()
PickSideApp_2.action = "MoveL"
PickSideApp_2.speed = 0.1
PickSideApp_2.movel.x = 0.0
PickSideApp_2.movel.y = 0.0
PickSideApp_2.movel.z = 0.05
PickSide = Action()
PickSide.action = "MoveL"
PickSide.speed = 0.1
PickSide.movel.x = 0.0
PickSide.movel.y = 0.0
PickSide.movel.z = -0.03

# Pick cube to CHECK TOP FACE:
PickTopApp = Action()
PickTopApp.action = "MoveL"
PickTopApp.speed = 0.1
PickTopApp.movel.x = 0.0
PickTopApp.movel.y = 0.0
PickTopApp.movel.z = 0.05
PickTop = Action()
PickTop.action = "MoveRP"
PickTop.speed = 0.1
PickTop.moverp.x = 0.0
PickTop.moverp.y = 0.0
PickTop.moverp.z = 0.19
PickTop.moverp.yaw = 0.0
PickTop.moverp.pitch = -20.0
PickTop.moverp.roll = 0.0

# Pick it back for PLACING:
RePickTopApp_PREV = Pose()
RePickTopApp_PREV.position.x = 0.528
RePickTopApp_PREV.position.y = 0.527
RePickTopApp_PREV.position.z = 1.057
RePickTopApp_PREV.orientation.x = 0.0
RePickTopApp_PREV.orientation.y = 0.938
RePickTopApp_PREV.orientation.z = 0.0
RePickTopApp_PREV.orientation.w = 0.346
RePickTop_PREV = Action()
RePickTop_PREV.action = "MoveL"
RePickTop_PREV.speed = 0.1
RePickTop_PREV.movel.x = 0.0
RePickTop_PREV.movel.y = 0.0
RePickTop_PREV.movel.z = -0.03
RePickTop = Action()
RePickTop.action = "MoveRP"
RePickTop.speed = 0.1
RePickTop.moverp.x = 0.0
RePickTop.moverp.y = 0.0
RePickTop.moverp.z = 0.19
RePickTop.moverp.yaw = 0.0
RePickTop.moverp.pitch = 20.0
RePickTop.moverp.roll = 0.0
RePickTopApp = Action()
RePickTopApp.action = "MoveL"
RePickTopApp.speed = 0.1
RePickTopApp.movel.x = 0.0
RePickTopApp.movel.y = 0.0
RePickTopApp.movel.z = 0.05

# ROTATION:
RotApp = Pose()
RotApp.position.x = 0.50
RotApp.position.y = 0.53
RotApp.position.z = 1.1
RotApp.orientation.x = 0.0
RotApp.orientation.y = 1.0
RotApp.orientation.z = 0.0
RotApp.orientation.w = 0.0
RotBack = Action()
RotBack.action = "MoveR"
RotBack.speed = 1.0
RotBack.mover.joint = "joint6"
RotBack.mover.value = 180.0
RotBack_other = Action()
RotBack_other.action = "MoveR"
RotBack_other.speed = 1.0
RotBack_other.mover.joint = "joint6"
RotBack_other.mover.value = -180.0
RotPlace = Action()
RotPlace.action = "MoveL"
RotPlace.speed = 0.1
RotPlace.movel.x = 0.0
RotPlace.movel.y = 0.0
RotPlace.movel.z = -0.03
RotationPick = Action()
RotationPick.action = "MoveRP"
RotationPick.speed = 0.1
RotationPick.moverp.x = 0.0
RotationPick.moverp.y = 0.0
RotationPick.moverp.z = 0.19
RotationPick.moverp.yaw = 0.0
RotationPick.moverp.pitch = 30.0
RotationPick.moverp.roll = 0.0
RotationPlace = Action()
RotationPlace.action = "MoveRP"
RotationPlace.speed = 0.1
RotationPlace.moverp.x = 0.0
RotationPlace.moverp.y = 0.0
RotationPlace.moverp.z = 0.19
RotationPlace.moverp.yaw = 0.0
RotationPlace.moverp.pitch = -30.0
RotationPlace.moverp.roll = 0.0
RotZ = Action()
RotZ.action = "MoveL"
RotZ.speed = 0.1
RotZ.movel.x = 0.0
RotZ.movel.y = 0.0
RotZ.movel.z = 0.03
RotX = Action()
RotX.action = "MoveL"
RotX.speed = 0.3
RotX.movel.x = 0.15
RotX.movel.y = 0.0
RotX.movel.z = 0.0

# =============================================================================== #
# ROS2 ActionClient for RobMove:

class RobMoveClient(Node):

    def __init__(self):

        super().__init__('irb120pe_RobMove_Client')
        self._action_client = ActionClient(self, Robmove, 'Robmove')

        print ("Waiting for /Robove ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print ("/Robmove ACTION SERVER detected.")

    def send_goal(self, TYPE, SPEED, TARGET_POSE):
        
        # 1. Assign variables:
        goal_msg = Robmove.Goal()
        goal_msg.type = TYPE
        goal_msg.speed = SPEED
        goal_msg.x = TARGET_POSE.position.x
        goal_msg.y = TARGET_POSE.position.y
        goal_msg.z = TARGET_POSE.position.z
        goal_msg.qx = TARGET_POSE.orientation.x
        goal_msg.qy = TARGET_POSE.orientation.y
        goal_msg.qz = TARGET_POSE.orientation.z
        goal_msg.qw = TARGET_POSE.orientation.w
        
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('RobMove ACTION CALL -> GOAL has been REJECTED.')
            return
        self.get_logger().info('RobMove ACTION CALL -> GOAL has been ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        # 1. Assign RESULT variable:
        RESULT = future.result().result
        RES.MESSAGE = RESULT.message
        RES.SUCCESS = RESULT.success
        
        # 2. Print RESULT:
        #print ("RobMove ACTION CALL finished.") 
        #print ("MESSAGE: " + RES.MESSAGE)
        #print ("SUCCESS: " + str(RES.SUCCESS))

# =============================================================================== #
# ROS2 ActionClient for Move:

class MoveClient(Node):

    def __init__(self):

        super().__init__('irb120pe_Move_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        print ("Waiting for /Robove ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print ("/Move ACTION SERVER detected.")

    def send_goal(self, ACTION):
        
        # 1. Assign variables:
        goal_msg = Move.Goal()
        goal_msg.action = ACTION.action
        goal_msg.speed = ACTION.speed
        goal_msg.movej = ACTION.movej
        goal_msg.mover = ACTION.mover
        goal_msg.movel = ACTION.movel
        goal_msg.movexyzw = ACTION.movexyzw
        goal_msg.movexyz = ACTION.movexyz
        goal_msg.moverot = ACTION.moverot
        goal_msg.moveypr = ACTION.moveypr
        goal_msg.moverp = ACTION.moverp
        goal_msg.moveg = ACTION.moveg
        
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Move ACTION CALL -> GOAL has been REJECTED.')
            return
        self.get_logger().info('Move ACTION CALL -> GOAL has been ACCEPTED.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        # 1. Assign RESULT variable:
        RESULT = future.result().result
        RES.MESSAGE = RESULT.result

        if "FAILED" in RES.MESSAGE:
            RES.SUCCESS = False
        else:
            RES.SUCCESS = True
        
        # 2. Print RESULT:
        #print ("Move ACTION CALL finished.") 
        #print ("MESSAGE: " + RES.MESSAGE)
        #print ("SUCCESS: " + str(RES.SUCCESS))

# =============================================================================== #
# ABB Robot I/O - ROS2 Service Client:

class abbRWS_IO(Node):

    def __init__(self):
        super().__init__('abbRWS_IO_client')
        self.cli = self.create_client(SetIOSignal, '/rws_client/set_io_signal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('Waiting for ABB-RWS I/O Service Server to be available...')
        print('ABB-RWS I/O Service Server detected.')
        self.req = SetIOSignal.Request()

    def send_request(self, signal, value):
        global RES_ABB
        self.req.signal = signal
        self.req.value = value
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        RES_ABB = self.future.result() 

# =============================================================================== #
# Declare ROS2 Init + MoveIt!2 global nodes:

# INITIALISE ROS NODE:
rclpy.init(args=None)

# GLOBAL VARIABLES -> INITIALISE RobMove+Move ACTION CLIENTs:
RobMove_CLIENT = RobMoveClient()
Move_CLIENT = MoveClient()
abbIO_CLIENT = abbRWS_IO()

# ===================================================================================== #
# ==================================== FUNCTIONS ====================================== #
# ===================================================================================== #

def toQuaternion(angle):

    # Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
    pi = 3.14159265358979
    k = pi/180.0

    # 1. Initial pose:
    Ax = 0.0
    Ay = 1.0
    Az = 0.0
    Aw = 0.0
    # 2. Get desired RELATIVE ROTATION:
    yaw = angle
    pitch = 0.0
    roll = 0.0
    cy = math.cos(k*yaw * 0.5)
    sy = math.sin(k*yaw * 0.5)
    cp = math.cos(k*pitch * 0.5)
    sp = math.sin(k*pitch * 0.5)
    cr = math.cos(k*roll * 0.5)
    sr = math.sin(k*roll * 0.5)
    Bx = sr * cp * cy - cr * sp * sy
    By = cr * sp * cy + sr * cp * sy
    Bz = cr * cp * sy - sr * sp * cy
    Bw = cr * cp * cy + sr * sp * sy
    # 3. Quaternion MULTIPLICATION:
    ROTATION = dict()
    ROTATION["ROTx"] = Aw*Bx + Ax*Bw + Ay*Bz - Az*By
    ROTATION["ROTy"] = Aw*By - Ax*Bz + Ay*Bw + Az*Bx
    ROTATION["ROTz"] = Aw*Bz + Ax*By - Ay*Bx + Az*Bw
    ROTATION["ROTw"] = Aw*Bw - Ax*Bx - Ay*By - Az*Bz

    print(angle)
    print(ROTATION)

    return(ROTATION)

def calibMoveIt2():

    global RobMove_CLIENT
    global Move_CLIENT
    global RES

    CALIB = Action()
    CALIB.action = "MoveR"
    CALIB.speed = 0.1
    CALIB.mover.joint = "joint1"
    CALIB.mover.value = 1.0
    Move_CLIENT.send_goal(CALIB)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    CALIB.mover.value = -1.0
    Move_CLIENT.send_goal(CALIB)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

def HomePos():

    global RobMove_CLIENT
    global Move_CLIENT
    global RES

    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = 1.0
    ACTION.movej.joint1 = 0.0
    ACTION.movej.joint2 = -30.0
    ACTION.movej.joint3 = 30.0
    ACTION.movej.joint4 = 0.0
    ACTION.movej.joint5 = 90.0
    ACTION.movej.joint6 = 0.0
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

def PickCube(x,y,ROTATION):

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    # ========================== #
    # 1. Move to -> PickApproach:

    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = y/1000 + 0.35
    TARGET_POSE.position.y = x/1000 + 0.2
    TARGET_POSE.position.z = 1.10
    TARGET_POSE.orientation.x = ROTATION["ROTx"]
    TARGET_POSE.orientation.y = ROTATION["ROTy"]
    TARGET_POSE.orientation.z = ROTATION["ROTz"]
    TARGET_POSE.orientation.w = ROTATION["ROTw"]

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2. Move to -> Pick:

    TYPE = "LIN"
    SPEED = 0.1
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = y/1000 + 0.35
    TARGET_POSE.position.y = x/1000 + 0.2
    TARGET_POSE.position.z = 1.10 - 0.03
    TARGET_POSE.orientation.x = ROTATION["ROTx"]
    TARGET_POSE.orientation.y = ROTATION["ROTy"]
    TARGET_POSE.orientation.z = ROTATION["ROTz"]
    TARGET_POSE.orientation.w = ROTATION["ROTw"]

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 3. Close GRIPPER:

    signal = "OpenGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "CloseGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Closed.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 4. Back to PickApproach:

    TYPE = "PTP"
    SPEED = 0.1
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = y/1000 + 0.35
    TARGET_POSE.position.y = x/1000 + 0.2
    TARGET_POSE.position.z = 1.10
    TARGET_POSE.orientation.x = ROTATION["ROTx"]
    TARGET_POSE.orientation.y = ROTATION["ROTy"]
    TARGET_POSE.orientation.z = ROTATION["ROTz"]
    TARGET_POSE.orientation.w = ROTATION["ROTw"]

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # CubePick -> Successful:
    RES.MESSAGE = "CUBE PICK SUCCESSFUL."
    RES.SUCCESS = True
    return(RES)

def CUBE_MID():

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    # ========================== #
    # 1. Move to MidApproach:

    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = PlaceMidApp

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2. Move to MidPlace:

    TYPE = "LIN"
    SPEED = 0.1
    TARGET_POSE = PlaceMid

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 3. Open GRIPPER:

    signal = "CloseGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "OpenGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Opened.")
    RES_ABB = "null"

    time.sleep(0.5)

    # PlaceMid -> Successful:
    RES.MESSAGE = "PLACE MID SUCCESSFUL."
    RES.SUCCESS = True
    return(RES)

def CHECK_TOP():

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    # ========================== #
    # 1. Place box in the middle of the workspace:
    RES = CUBE_MID()

    if (RES.SUCCESS == False):
        rclpy.shutdown()
        print(RES.MESSAGE)
        print("CLOSING PROGRAM...")
        return(RES)

    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2.1 Rotate around the cube -> MoveRP:
    ACTION = PickTop
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2.2 Rotate around the cube -> MoveRP:
    ACTION = PickTop
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 3. Close GRIPPER:

    signal = "OpenGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "CloseGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Closed.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 4. Lift cube a bit:
    ACTION = PickTopApp
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 5. Place in front of CAMERA:
    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = FacePose_4

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    
    # CheckTop -> Successful:
    RES.MESSAGE = "CheckTop successfully finished."
    RES.SUCCESS = True
    return(RES)  

def RePick_TOP():

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    # ========================== #
    # 1. Move to RePick Approach position:
    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = RePickTopApp_PREV

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2. Move down a bit:
    ACTION = RePickTop_PREV
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 3. Open GRIPPER:

    signal = "CloseGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "OpenGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Opened.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 4.1 Rotate around the cube -> MoveRP:
    ACTION = RePickTop
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 4.2 Rotate around the cube -> MoveRP:
    ACTION = RePickTop
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 5. Close GRIPPER:

    signal = "OpenGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "CloseGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Closed.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 6. Lift cube a bit:
    ACTION = RePickTopApp
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    if (RES.SUCCESS == False):
        return(RES)
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    
    # CheckTop -> Successful:
    RES.MESSAGE = "RePickTop successfully finished."
    RES.SUCCESS = True
    return(RES)
    

def CHECK_SIDES(camera):

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    RESULT = dict()

    # ========================== #
    # 1. Move to CHECK-FRONT:
    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = FacePose_1

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        RESULT["RES"] = RES
        RESULT["COLOUR"] = "None"
        RESULT["FACE"] = "None"
        return(RESULT)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2. DetectColour:
    COLOUR = detectColour(camera)
    if (COLOUR != "CUBE"):
        RES.SUCCESS = True
        RES.MESSAGE = "CHECK_SIDES successfully finished."
        RESULT["RES"] = RES
        RESULT["COLOUR"] = COLOUR
        RESULT["FACE"] = "FRONT"
        return(RESULT)

    calibMoveIt2()

    # ========================== #
    # 3 Rotate to BACK side:
    ACTION = FacePose_2
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    
    if (RES.SUCCESS == False):

        RES.MESSAGE = "null"
        RES.SUCCESS = False
        
        ACTION = FacePose_2_other
        Move_CLIENT.send_goal(ACTION)
        while rclpy.ok():
            rclpy.spin_once(Move_CLIENT)
            if (RES.MESSAGE != "null"):
                break
        
        if (RES.SUCCESS == False):
            RESULT["RES"] = RES
            RESULT["COLOUR"] = "None"
            RESULT["FACE"] = "None"
            return(RESULT)

    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 4. DetectColour:
    COLOUR = detectColour(camera)
    if (COLOUR != "CUBE"):
        RES.SUCCESS = True
        RES.MESSAGE = "CHECK_SIDES successfully finished."
        RESULT["RES"] = RES
        RESULT["COLOUR"] = COLOUR
        RESULT["FACE"] = "BACK"
        return(RESULT)

    calibMoveIt2()

    # ========================== #
    # 5. Move to CHECK-BOTTOM:
    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = FacePose_3

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):

        MoveMidSAFE()

        RESULT["RES"] = RES
        RESULT["COLOUR"] = "None"
        RESULT["FACE"] = "None"
        return(RESULT)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 6. DetectColour:
    COLOUR = detectColour(camera)
    if (COLOUR != "CUBE"):
        
        calibMoveIt2()
        MoveMidSAFE()

        RES.SUCCESS = True
        RES.MESSAGE = "CHECK_SIDES successfully finished."
        RESULT["RES"] = RES
        RESULT["COLOUR"] = COLOUR
        RESULT["FACE"] = "BOTTOM"
        return(RESULT)

    elif (COLOUR == "CUBE"):

        calibMoveIt2()
        MoveMidSAFE()

        RES.SUCCESS = True
        RES.MESSAGE = "CHECK_SIDES successfully finished."
        RESULT["RES"] = RES
        RESULT["COLOUR"] = COLOUR
        RESULT["FACE"] = "NONE"
        return(RESULT)


def MoveMidSAFE():

    global RobMove_CLIENT
    global RES

    RES.MESSAGE = "null"
    RES.SUCCESS = False

    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = FacePose_1

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

def Check_LR(camera):

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    RESULT = dict()

    CUBE_MID()
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 1. Move UP a bit:
    TYPE = "LIN"
    SPEED = 0.3
    TARGET_POSE = PlaceMidApp

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        RESULT["RES"] = RES
        RESULT["COLOUR"] = "None"
        RESULT["FACE"] = "None"
        return(RESULT)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2. ROTATE:
    ACTION = PickSideApp_1
    Move_CLIENT.send_goal(ACTION)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    
    if (RES.SUCCESS == False):

        RES.MESSAGE = "null"
        RES.SUCCESS = False
        
        ACTION = PickSideApp_1_other
        Move_CLIENT.send_goal(ACTION)
        while rclpy.ok():
            rclpy.spin_once(Move_CLIENT)
            if (RES.MESSAGE != "null"):
                break
        
        if (RES.SUCCESS == False):
            RESULT["RES"] = RES
            RESULT["COLOUR"] = "None"
            RESULT["FACE"] = "None"
            return(RESULT)

    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 3. Move DOWN a bit:
    ACTION = PickSide
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        RESULT["RES"] = RES
        RESULT["COLOUR"] = "None"
        RESULT["FACE"] = "None"
        return(RESULT)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 4. Close GRIPPER:

    signal = "OpenGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "CloseGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Closed.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 5. LIFT cube:
    ACTION = PickSideApp_2
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        RESULT["RES"] = RES
        RESULT["COLOUR"] = "None"
        RESULT["FACE"] = "None"
        return(RESULT)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False


def detectColour(camera):

    # 1. Initialize model
    modelpath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'yolov8')
    model = YOLO(modelpath + '/best.pt')  # Pretrained YOLOv8n model.
    names = model.names 

    # 2. Detect colour
    T = time.time() + 1.5

    StickerCount = 0
    WhiteCount = 0
    BlackCount = 0
    BlueCount = 0
    
    while (time.time() <= T):
        ret, colourImg = camera.read()
        #cv2.imshow("Detect colour", colourImg)
        results = model(colourImg)
        #annotated_frame =results[0].plot()
        boxes = results[0].boxes
        ids = []
        for box in boxes:
            box = boxes.xyxy
            #print("Box cordinates: ", boxes.xyxy)
            for c in boxes.cls:
                ids.append(names[int(c)])

        for ID in ids:
            if (ID == "sticker"):
                StickerCount = StickerCount + 1
            elif (ID == "white"):
                WhiteCount = WhiteCount + 1
            elif (ID == "black"):
                BlackCount = BlackCount + 1
            elif (ID == "blue"):
                BlueCount = BlueCount + 1

    if (WhiteCount == BlueCount == BlackCount == 0 and StickerCount > 150):
        COLOUR = "CUBE"
    elif (WhiteCount > BlueCount and WhiteCount > BlackCount):
        COLOUR = "WHITE"
    elif (WhiteCount < BlueCount and BlueCount > BlackCount):
        COLOUR = "BLUE"
    elif (BlackCount > BlueCount and WhiteCount < BlackCount):
        COLOUR = "BLACK"
    else:
        COLOUR = "CUBE"

    return(COLOUR)

def RotateCube(FACE):

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    # ========================== #
    # 1. Move to MidApproach:

    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = RotApp

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 2. If sticker at back, rotate 180:
    if (FACE == "BACK"):

        ACTION = RotBack
        Move_CLIENT.send_goal(ACTION)
        while rclpy.ok():
            rclpy.spin_once(Move_CLIENT)
            if (RES.MESSAGE != "null"):
                break
        
        if (RES.SUCCESS == False):

            RES.MESSAGE = "null"
            RES.SUCCESS = False
            
            ACTION = RotBack_other
            Move_CLIENT.send_goal(ACTION)
            while rclpy.ok():
                rclpy.spin_once(Move_CLIENT)
                if (RES.MESSAGE != "null"):
                    break
            
            if (RES.SUCCESS == False):
                return(RES)

        print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

    # ========================== #
    # 3. Move to MidPlace:

    ACTION = RotPlace
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 4. Open GRIPPER:

    signal = "CloseGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "OpenGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Opened.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 5. 180 degrees back:
    if (FACE == "BACK"):

        # 5.1. Move up:
        ACTION = RotZ
        Move_CLIENT.send_goal(ACTION)
        
        while rclpy.ok():
            rclpy.spin_once(Move_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            return(RES)
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        # 5.2. 180 deg rotate:
        ACTION = RotBack_other
        Move_CLIENT.send_goal(ACTION)
        while rclpy.ok():
            rclpy.spin_once(Move_CLIENT)
            if (RES.MESSAGE != "null"):
                break
        
        if (RES.SUCCESS == False):

            RES.MESSAGE = "null"
            RES.SUCCESS = False
            
            ACTION = RotBack
            Move_CLIENT.send_goal(ACTION)
            while rclpy.ok():
                rclpy.spin_once(Move_CLIENT)
                if (RES.MESSAGE != "null"):
                    break
            
            if (RES.SUCCESS == False):
                return(RES)

        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        # ========================== #
        # 5.3. Down:
        ACTION = RotPlace
        Move_CLIENT.send_goal(ACTION)
        
        while rclpy.ok():
            rclpy.spin_once(Move_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            return(RES)

        print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

    # ========================== #
    # 6.1. ROTATE CUBE once:
    if (FACE != "BOTTOM"):
        
        RES = ROTCube()
        if (RES.SUCCESS == True):
            RES.MESSAGE = "RotateCube successfully finished."
            return(RES)
        else:
            RES.MESSAGE = "RotateCube -> ERROR."
            return(RES)
    
    # ========================== #
    # 6.2. ROTATE CUBE twice:
    if (FACE == "BOTTOM"):

        RES = ROTCube()
        if (RES.SUCCESS == True):

            RES.MESSAGE = "null"
            RES.SUCCESS = False
            
            # ========================== #
            # 6.2.1. Move to MidPlace:

            TYPE = "PTP"
            SPEED = 0.3
            TARGET_POSE = RotApp

            RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
            
            while rclpy.ok():
                rclpy.spin_once(RobMove_CLIENT)
                if (RES.MESSAGE != "null"):
                    break

            if (RES.SUCCESS == False):
                return(RES)
            
            print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
            print("ACTION CALL successful? -> " + str(RES.SUCCESS))
            print("")
            RES.MESSAGE = "null"
            RES.SUCCESS = False

            # ========================== #
            # 6.2.2. Move to MidPlace (down a bit):

            ACTION = RotPlace
            Move_CLIENT.send_goal(ACTION)
            
            while rclpy.ok():
                rclpy.spin_once(Move_CLIENT)
                if (RES.MESSAGE != "null"):
                    break

            if (RES.SUCCESS == False):
                return(RES)
            
            print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
            print("ACTION CALL successful? -> " + str(RES.SUCCESS))
            print("")
            RES.MESSAGE = "null"
            RES.SUCCESS = False

            time.sleep(0.2)

            # ========================== #
            # 6.2.3. Open GRIPPER:

            signal = "CloseGripper"
            value = "0"
            abbIO_CLIENT.send_request(signal,value)
            signal = "OpenGripper"
            value = "1"
            abbIO_CLIENT.send_request(signal,value)
            print ("Result -> Gripper Opened.")
            RES_ABB = "null"

            time.sleep(0.5)

            # ========================== #
            # 6.2.4. ROTATE AGAIN:
            RES = ROTCube()

            if (RES.SUCCESS == True):
                RES.MESSAGE = "RotateCube successfully finished."
                return(RES)
            else:
                RES.MESSAGE = "RotateCube -> ERROR."
                return(RES)

        else:
            RES.MESSAGE = "RotateCube -> ERROR."
            return(RES)


def ROTCube():

    global RobMove_CLIENT
    global Move_CLIENT
    global abbIO_CLIENT
    global RES

    # ========================== #
    # 1. ROT 2x30deg (RP):

    ACTION = RotationPick
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    ACTION = RotationPick
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 2. Close GRIPPER:

    signal = "OpenGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "CloseGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Closed.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 3. Move up:
    ACTION = RotZ
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 4. ROT 2x30deg (RP):

    ACTION = RotationPlace
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    ACTION = RotationPlace
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 5. Move X:
    ACTION = RotX
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 6. ROT 1x30deg (RP):

    ACTION = RotationPlace
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 7. Move down:
    ACTION = RotPlace
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 8. Open GRIPPER:

    signal = "CloseGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "OpenGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Opened.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 9. Move up:
    ACTION = RotZ
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 10. ROT 2x30deg (RP):

    ACTION = RotationPick
    Move_CLIENT.send_goal(ACTION)
    9
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ======null==================== #
    # 11. Move down:
    ACTION = RotPlace
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    time.sleep(0.2)

    # ========================== #
    # 12. Close GRIPPER:

    signal = "OpenGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "CloseGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Closed.")
    RES_ABB = "null"

    time.sleep(0.5)

    # ========================== #
    # 13. Move up:
    ACTION = RotZ
    Move_CLIENT.send_goal(ACTION)
    
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break

    if (RES.SUCCESS == False):
        return(RES)
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")

    RES.MESSAGE = "RotCube successfully finished."
    RES.SUCCESS = True
    return(RES)


# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def main(args=None):

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ABB IRB120: Pose Estimation for CUBE Pick&Place")
    print("Python script -> yolo.py")
    print("")
    
    # Import GLOBAL VARIABLES -> RES, RES_PE:
    global RES
    global RES_ABB

    # 2. INITIALISE RobMove+Move ACTION CLIENTs:
    global RobMove_CLIENT
    global Move_CLIENT 

    # 3. INITIALISE ABB I/O Client:
    global abbIO_CLIENT

    # ===================================================================================== #
    # HOME -> Gripper Open + HomePos:

    # OPEN GRIPPER:
    signal = "CloseGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "OpenGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Opened.")
    RES_ABB = "null"

    time.sleep(0.5)

    # SMALL CALIBRATION (MoveIt!2) ROUTINE:
    calibMoveIt2()

    # MOVE TO HOMEPOSE:
    HomePos()

    # ===================================================================================== #

    # ===================================================================================== #
    # Yolo: CALIBRATION + POSE ESTIMATION:

    camera = cv2.VideoCapture(0) # Define the camera and its port.

    while True:
        ret, inputImg = camera.read()
        cv2.imshow("Input Image", inputImg)
        key = cv2.waitKey(1)
        if key == ord('q') or inputImg is None:
            break    
    
    if not ret:
        print("Error! Failed at capturing the image")
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()

    if inputImg is None:
        # Error!
        print("Error! Image empty")
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()

    # Values of the calibration x and y in mm:
    w = 700
    h = 400

    # Release camera
    #camera.release

    # Detection of the Aruco Markers in the input image:
    param = cv2.aruco.DetectorParameters()
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    detector = aruco.ArucoDetector(aruco_dict, param)
    markerCorners, markerIds, rejectedCandidates =detector.detectMarkers(inputImg)
        
    # Visualize detection of the Aruco markers:
    outputImage = inputImg.copy()
    cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds)

    # Calibration process:
    src = np.zeros((4, 2), dtype=np.float32)  # Points of the corners from the input image.
    dst = np.array([[0.0, 0.0], [w, 0.0], [0.0, h], [w, h]], dtype=np.float32)  # Points calibrated with w and h.
    
    poly_corner = np.zeros((4, 3), dtype=np.int32)

    # Get the first corner & ID of the markers:
    for i in range(4):
        poly_corner[i][0] = int(markerIds[i][0])
        poly_corner[i][1] = int(markerCorners[i][0][0][0])
        poly_corner[i][2] = int(markerCorners[i][0][0][1])
    
    # Arrange by ID: 
    for i in range(3):
        for j in range(i + 1, 4):
            if poly_corner[i][0] > poly_corner[j][0]:
                for k in range(3):
                    aux = poly_corner[i][k]
                    poly_corner[i][k] = poly_corner[j][k]
                    poly_corner[j][k] = aux
    
    # Get the source vector:
    for i in range(4):
        src[i] = np.array([poly_corner[i][1], poly_corner[i][2]], dtype=np.float32)
    
    # Get the transform matrix:
    perspTransMatrix = cv2.getPerspectiveTransform(src, dst)

    # Get image of the perspective:
    perspectiveImg = cv2.warpPerspective(outputImage, perspTransMatrix, (int(w), int(h)))
    cv2.imshow("Perspective Image", perspectiveImg)
    modelpath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'yolov8')

    # Load a model:
    model = YOLO(modelpath + '/best.pt')  # Pretrained YOLOv8n model.
    names = model.names

    # Get the results:
    results = model(perspectiveImg)  # Return a list of Results objects.
    annotated_frame =results[0].plot()

    # Process results list:
    boxes = results[0].boxes
    ids = []
    for box in boxes:
        box = boxes.xyxy
        print("Box cordinates: ", boxes.xyxy)
        for c in boxes.cls:
            ids.append(names[int(c)])
    
    print("The ids detected: ", ids)
    x1, y1, x2, y2 = boxes[0].xyxy[0]

    x = int(x1)-5
    y = int(y1)-5
    w = int(x2)+5
    h = int(y2)+5

    classif = boxes[0].cls

    ROI = perspectiveImg[y:h, x:w]

    # Pose estimation:
    ROI = cv2.GaussianBlur(ROI,(3,3),0)
    imgHSV = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)
    lowLimit = (0, 10, 10)
    upLimit = (70, 255, 255)
    mask = cv2.inRange(imgHSV, lowLimit, upLimit)

    # find the contours in the dilated image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    poly_contour = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 700:
            poly_contour.append(contour)
    # Get the polygonal approximation of the contour.

    if not poly_contour:
        print("Contour not detected.")
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()
    
    cv2.drawContours(ROI, poly_contour, -1, (0, 255, 0), 1)
    
    for cnt in poly_contour:
        rect = cv2.minAreaRect(cnt)
        (xo, yo), (wo, ho), angle = rect
        xo = xo + x
        yo = yo + y
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.polylines(ROI, [box], True, (255,0,0), 2)
    
    print(xo)
    print(yo)

    rotation = False

    if (angle < 80 and angle > 10) or (angle > -80 and angle < -10):
        rotation = True 

    print("Angle: ", angle)
    if rotation:
        print("Rotated")
        ROTATION = toQuaternion(angle)

    else:
        print("Horizontal")
        ROTATION = {"ROTx": 0.0, "ROTy": 1.0, "ROTz": 0.0, "ROTw": 0.0}


    #cv2.imshow("Edges", mask)
    #cv2.imshow("Contours", ROI)
    #cv2.waitKey(0)

    #cv2.destroyAllWindows()
    #time.sleep(1)

    # ===================================================================================== #

    # ===================================================================================== #
    # ROBOT MOVEMENT:

    # ========================== #
    # 0. Small CALIB movement:
    calibMoveIt2()

    # 1. PICK CUBE:
    RES = PickCube(xo, yo, ROTATION)

    if (RES.SUCCESS == False):
        rclpy.shutdown()
        print(RES.MESSAGE)
        print("CLOSING PROGRAM...")
        exit()
    else:
        print(RES.MESSAGE)
        RES.MESSAGE = "null"
        RES.SUCCESS = False
    

    # ============================= #
    # LOOP --> Find STICKER in CUBE #
    # ============================= #

    COUNT = 0

    BREAK = False
    while BREAK == False:

        # CHECK (TOP FACE, detected at the beginning) -> STICKER/CUBE:
        for ID in ids:

            # If STICKER on TOP -> Detect colour and PLACE:
            if ((ID == "sticker") or (ID == "white") or (ID == "black") or (ID == "blue")):

                CHECK_TOP()

                if (RES.SUCCESS == False):
                    rclpy.shutdown()
                    print(RES.MESSAGE)
                    print("CLOSING PROGRAM...")
                    exit()
                else:
                    print(RES.MESSAGE)
                    RES.MESSAGE = "null"
                    RES.SUCCESS = False

                    COLOUR = detectColour(camera)

                    RePick_TOP()

                    if (RES.SUCCESS == False):
                        rclpy.shutdown()
                        print(RES.MESSAGE)
                        print("CLOSING PROGRAM...")
                        exit()
                    else:
                        print(RES.MESSAGE)
                        RES.MESSAGE = "null"
                        RES.SUCCESS = False

                        BREAK = True
                        NEEDROT = False
                
                break

            # If STICKER is NOT on TOP -> Find where the sticker is -> CHECK other 5 sides:
            else:

                SIDES_RES = CHECK_SIDES(camera)
                
                if (SIDES_RES["RES"].SUCCESS == False):
                    rclpy.shutdown()
                    print(RES.MESSAGE)
                    print("CLOSING PROGRAM...")
                    exit()
                
                elif(SIDES_RES["COLOUR"] != "CUBE"):
                    
                    print(SIDES_RES)
                    RES.MESSAGE = "null"
                    RES.SUCCESS = False
                    HomePos()
                    
                    COLOUR = SIDES_RES["COLOUR"]

                    BREAK = True
                    NEEDROT = True

                
                elif(SIDES_RES["COLOUR"] == "CUBE"):

                    if (COUNT == 2):
                        BREAK = True
                        NEEDROT = False
                    
                    RES.MESSAGE = "null"
                    RES.SUCCESS = False
                    Check_LR(camera)

                    COUNT = COUNT + 1
                
                break
    
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ROTATE if necessary, in order to put sticker on top:
    if NEEDROT:

        RES = RotateCube(SIDES_RES["FACE"])

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()

        RES.MESSAGE = "null"
        RES.SUCCESS = False
    
    # PLACE:
    if (COLOUR == "BLACK"):

        # ========================== #
        # Move to BlackApproach:

        TYPE = "PTP"
        SPEED = 0.3
        TARGET_POSE = PlaceBLACK_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        # ========================== #
        # Move to BlackPlace:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceBLACK

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        time.sleep(0.2)

        # ========================== #
        # Open GRIPPER:

        signal = "CloseGripper"
        value = "0"
        abbIO_CLIENT.send_request(signal,value)
        signal = "OpenGripper"
        value = "1"
        abbIO_CLIENT.send_request(signal,value)
        print ("Result -> Gripper Opened.")
        RES_ABB = "null"

        time.sleep(0.5)

        # ========================== #
        # Move to BlackApproach:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceBLACK_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False
    
    elif (COLOUR == "BLUE"):

        # ========================== #
        # Move to BlueApproach:

        TYPE = "PTP"
        SPEED = 0.3
        TARGET_POSE = PlaceBLUE_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        # ========================== #
        # Move to BLUEPlace:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceBLUE

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        time.sleep(0.2)

        # ========================== #
        # Open GRIPPER:

        signal = "CloseGripper"
        value = "0"
        abbIO_CLIENT.send_request(signal,value)
        signal = "OpenGripper"
        value = "1"
        abbIO_CLIENT.send_request(signal,value)
        print ("Result -> Gripper Opened.")
        RES_ABB = "null"

        time.sleep(0.5)

        # ========================== #
        # Move to BLUEApproach:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceBLUE_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

    elif (COLOUR == "WHITE"):

        # ========================== #
        # Move to WHITEApproach:

        TYPE = "PTP"
        SPEED = 0.3
        TARGET_POSE = PlaceWHITE_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        # ========================== #
        # Move to WHITEPlace:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceWHITE

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        time.sleep(0.2)

        # ========================== #
        # Open GRIPPER:

        signal = "CloseGripper"
        value = "0"
        abbIO_CLIENT.send_request(signal,value)
        signal = "OpenGripper"
        value = "1"
        abbIO_CLIENT.send_request(signal,value)
        print ("Result -> Gripper Opened.")
        RES_ABB = "null"

        time.sleep(0.5)

        # ========================== #
        # Move to WHITEApproach:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceWHITE_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

    elif (COLOUR == "CUBE"):

        # ========================== #
        # Move to CUBEApproach:

        TYPE = "PTP"
        SPEED = 0.3
        TARGET_POSE = PlaceCUBE_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        # ========================== #
        # Move to CUBEPlace:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceCUBE

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

        time.sleep(0.2)

        # ========================== #
        # Open GRIPPER:

        signal = "CloseGripper"
        value = "0"
        abbIO_CLIENT.send_request(signal,value)
        signal = "OpenGripper"
        value = "1"
        abbIO_CLIENT.send_request(signal,value)
        print ("Result -> Gripper Opened.")
        RES_ABB = "null"

        time.sleep(0.5)

        # ========================== #
        # Move to CUBEApproach:

        TYPE = "LIN"
        SPEED = 0.1
        TARGET_POSE = PlaceCUBE_app

        RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
        
        while rclpy.ok():
            rclpy.spin_once(RobMove_CLIENT)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == False):
            rclpy.shutdown()
            print(RES.MESSAGE)
            print("CLOSING PROGRAM...")
            exit()
        
        print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
        print("ACTION CALL successful? -> " + str(RES.SUCCESS))
        print("")
        RES.MESSAGE = "null"
        RES.SUCCESS = False

    HomePos()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


# FOR MoveRP:
# ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.00, y: 0.00, z: 0.19, yaw: 0.00, pitch: {}, roll: 0.00}, speed: 1.0}"
