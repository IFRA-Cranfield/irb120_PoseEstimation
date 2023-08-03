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

# Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
pi = 3.14159265358979
k = pi/180.0

# =============================================================================== #
# PRE-DEFINED ROBOT POSES:

PlacePose_1 = Pose()
PlacePose_1.position.x = 0.0
PlacePose_1.position.y = 0.0
PlacePose_1.position.z = 0.0
PlacePose_1.orientation.x = 0.0
PlacePose_1.orientation.y = 0.0
PlacePose_1.orientation.z = 0.0
PlacePose_1.orientation.w = 0.0
PlacePose_1_TYPE = "PTP"
PlacePose_1_SPEED = 0.5
PlaceAppPose_1 = Pose()
PlaceAppPose_1.position.x = 0.0
PlaceAppPose_1.position.y = 0.0
PlaceAppPose_1.position.z = 0.0
PlaceAppPose_1.orientation.x = 0.0
PlaceAppPose_1.orientation.y = 0.0
PlaceAppPose_1.orientation.z = 0.0
PlaceAppPose_1.orientation.w = 0.0
PlaceAppPose_1_TYPE = "PTP"
PlaceAppPose_1_SPEED = 0.5

PlacePose_2 = Pose()
PlacePose_2.position.x = 0.0
PlacePose_2.position.y = 0.0
PlacePose_2.position.z = 0.0
PlacePose_2.orientation.x = 0.0
PlacePose_2.orientation.y = 0.0
PlacePose_2.orientation.z = 0.0
PlacePose_2.orientation.w = 0.0
PlacePose_2_TYPE = "PTP"
PlacePose_2_SPEED = 0.5
PlaceAppPose_2 = Pose()
PlaceAppPose_2.position.x = 0.0
PlaceAppPose_2.position.y = 0.0
PlaceAppPose_2.position.z = 0.0
PlaceAppPose_2.orientation.x = 0.0
PlaceAppPose_2.orientation.y = 0.0
PlaceAppPose_2.orientation.z = 0.0
PlaceAppPose_2.orientation.w = 0.0
PlaceAppPose_2_TYPE = "PTP"
PlaceAppPose_2_SPEED = 0.5

PlacePose_3 = Pose()
PlacePose_3.position.x = 0.0
PlacePose_3.position.y = 0.0
PlacePose_3.position.z = 0.0
PlacePose_3.orientation.x = 0.0
PlacePose_3.orientation.y = 0.0
PlacePose_3.orientation.z = 0.0
PlacePose_3.orientation.w = 0.0
PlacePose_3_TYPE = "PTP"
PlacePose_3_SPEED = 0.5
PlaceAppPose_3 = Pose()
PlaceAppPose_3.position.x = 0.0
PlaceAppPose_3.position.y = 0.0
PlaceAppPose_3.position.z = 0.0
PlaceAppPose_3.orientation.x = 0.0
PlaceAppPose_3.orientation.y = 0.0
PlaceAppPose_3.orientation.z = 0.0
PlaceAppPose_3.orientation.w = 0.0
PlaceAppPose_3_TYPE = "PTP"
PlaceAppPose_3_SPEED = 0.5

# Check FRONTAL FACE:
FacePose_1 = Pose()
FacePose_1.position.x = 0.501
FacePose_1.position.y = 0.525
FacePose_1.position.z = 1.435
FacePose_1.orientation.x = 0.0
FacePose_1.orientation.y = 0.708
FacePose_1.orientation.z = 0.0
FacePose_1.orientation.w = 0.707
# Check BACK FACE:
FacePose_2 = Action()
FacePose_2.action = "MoveR"
FacePose_2.speed = 1.0
FacePose_2.mover.joint = "Joint6"
FacePose_2.mover.value = 180.0
# Check BOTTOM FACE:
FacePose_3 = Pose()
FacePose_3.position.x = 0.62
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
PickSideApp_1.mover.joint = "Joint6"
PickSideApp_1.mover.value = 90.0
PickSideApp_2 = Action()
PickSideApp_2.action = "MoveL"
PickSideApp_2.speed = 0.1
PickSideApp_2.movel.x = 0.0
PickSideApp_2.movel.y = 0.0
PickSideApp_2.movel.z = 0.0 #+{}
PickSide = Action()
PickSide.action = "MoveL"
PickSide.speed = 0.1
PickSide.movel.x = 0.0
PickSide.movel.y = 0.0
PickSide.movel.z = 0.0 #-{}

# Pick cube to CHECK TOP FACE:
PickTopApp = Action()
PickTopApp.action = "MoveL"
PickTopApp.speed = 0.1
PickTopApp.movel.x = 0.0
PickTopApp.movel.y = 0.0
PickTopApp.movel.z = 0.0 #+{}
PickTop = Action()
PickTop.action = "MoveRP"
PickTop.speed = 0.1
PickTop.moverp.x = 0.0
PickTop.moverp.y = 0.0
PickTop.moverp.z = 0.19
PickTop.moverp.yaw = 0.0
PickTop.moverp.pitch = -20.0
PickTop.moverp.roll = 0.0

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

    # 1. INITIALISE ROS NODE:
    rclpy.init(args=args)

    # 2. INITIALISE RobMove+Move ACTION CLIENTs:
    RobMove_CLIENT = RobMoveClient()
    Move_CLIENT = MoveClient()

    # 3. INITIALISE ABB I/O Client:
    abbIO_CLIENT = abbRWS_IO()

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
    
    CALIB = Action()
    CALIB.action = "MoveR"
    CALIB.speed = 0.1
    CALIB.mover.joint = "joint1"
    CALIB.mover.value = 15.0
    Move_CLIENT.send_goal(CALIB)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    CALIB.mover.value = -15.0
    Move_CLIENT.send_goal(CALIB)
    while rclpy.ok():
        rclpy.spin_once(Move_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # MOVE TO HOMEPOSE:
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
    print("RESULT of Move ACTION CALL: " + RES.MESSAGE)
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

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

    if inputImg is None:
        # Error!
        print("Error! Image empty")
        rclpy.shutdown()

    # Values of the calibration x and y in mm:
    w = 700
    h = 400

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
            ids = names[int(c)]
    
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

    if (angle < 70 and angle > 20) or (angle > -70 and angle < -20):
        rotation = True 

    print("Angle: ", angle)
    if rotation:
        print("Rotated")

        # QUATERNION CONVERSION:
        # 1. Initial pose:
        Ax = 0.0
        Ay = 1.0
        Az = 0.0
        Aw = 0.0
        # 2. Get desired RELATIVE ROTATION:
        yaw = angle*k
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
        ROTw = Aw*Bw - Ax*Bx - Ay*By - Az*Bz
        ROTx = Aw*Bx + Ax*Bw + Ay*Bz - Az*By
        ROTy = Aw*By - Ax*Bz + Ay*Bw + Az*Bx
        ROTz = Aw*Bz + Ax*By - Ay*Bx + Az*Bw

    else:
        print("Horizontal")

        ROTw = 0.0
        ROTx = 0.0
        ROTy = 1.0
        ROTz = 0.0

    #cv2.imshow("Edges", mask)
    #cv2.imshow("Contours", ROI)
    #cv2.waitKey(0)

    #cv2.destroyAllWindows()
    #time.sleep(1)

    # ===================================================================================== #

    # ===================================================================================== #
    # ROBOT MOVEMENT:

    # ========================== #
    # 1. Move to -> PickApproach:

    TYPE = "PTP"
    SPEED = 0.3
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = yo/1000 + 0.35
    TARGET_POSE.position.y = xo/1000 + 0.2
    TARGET_POSE.position.z = 1.10
    TARGET_POSE.orientation.x = ROTx
    TARGET_POSE.orientation.y = ROTy
    TARGET_POSE.orientation.z = ROTz
    TARGET_POSE.orientation.w = ROTw

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    
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
    TARGET_POSE.position.x = yo/1000 + 0.35
    TARGET_POSE.position.y = xo/1000 + 0.2
    TARGET_POSE.position.z = 1.10 - 0.03
    TARGET_POSE.orientation.x = ROTx
    TARGET_POSE.orientation.y = ROTy
    TARGET_POSE.orientation.z = ROTz
    TARGET_POSE.orientation.w = ROTw

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    
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
    TARGET_POSE.position.x = yo/1000 + 0.35
    TARGET_POSE.position.y = xo/1000 + 0.2
    TARGET_POSE.position.z = 1.10
    TARGET_POSE.orientation.x = ROTx
    TARGET_POSE.orientation.y = ROTy
    TARGET_POSE.orientation.z = ROTz
    TARGET_POSE.orientation.w = ROTw

    RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    while rclpy.ok():
        rclpy.spin_once(RobMove_CLIENT)
        if (RES.MESSAGE != "null"):
            break
    
    print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    print("ACTION CALL successful? -> " + str(RES.SUCCESS))
    print("")
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    # ========================== #
    # 5.1. IF: 

    rclpy.shutdown()

if __name__ == '__main__':
    main()


# FOR MoveRP:
# ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.00, y: 0.00, z: 0.19, yaw: 0.00, pitch: {}, roll: 0.00}, speed: 1.0}"
