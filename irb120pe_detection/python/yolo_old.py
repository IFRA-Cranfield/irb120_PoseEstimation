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

#Declaration of GLOBAL VARIABLES --> CONSTANT VALUES for angle transformation (DEG->RAD):
pi = 3.14159265358979
k = pi/180.0

# =============================================================================== #
# ROS2 ActionClient for RobMove:

class MoveClient(Node):

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
        print ("RobMove ACTION CALL finished.") 
        print ("MESSAGE: " + RES.MESSAGE)
        print ("SUCCESS: " + str(RES.SUCCESS))

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

    # 2. INITIALISE RobMove ACTION CLIENT:
    RobMove_CLIENT = MoveClient()

    # 3. INITIALISE ABB I/O Client:
    abbIO_CLIENT = abbRWS_IO()

    # MOVE TO HOME POSITION + OpenGripper:

    signal = "CloseGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "OpenGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Opened.")
    RES_ABB = "null"

    TYPE = "PTP"
    SPEED = 0.5
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = 0.275
    TARGET_POSE.position.y = 0.876
    TARGET_POSE.position.z = 1.438
    TARGET_POSE.orientation.x = 0.658
    TARGET_POSE.orientation.y = -0.658
    TARGET_POSE.orientation.z = -0.259
    TARGET_POSE.orientation.w = -0.259

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

    # ============================ CALIBRATION + POSE ESTIMATION ============================ #

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

    cv2.imshow("Edges", mask)
    cv2.imshow("Contours", ROI)
    cv2.waitKey(0)

    cv2.destroyAllWindows()
    time.sleep(1)

    # ============================ CALIBRATION + POSE ESTIMATION ============================ #

    # === Call RobMove ACTION === #

    TYPE = "PTP"
    SPEED = 0.5
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = yo/1000 +0.35
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

    TYPE = "LIN"
    SPEED = 0.1
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = yo/1000 +0.35
    TARGET_POSE.position.y = xo/1000 + 0.2
    TARGET_POSE.position.z = 1.07
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

    # === Call RobMove ACTION === #

    # === Close GRIPPER === #

    time.sleep(0.01)

    signal = "OpenGripper"
    value = "0"
    abbIO_CLIENT.send_request(signal,value)
    signal = "CloseGripper"
    value = "1"
    abbIO_CLIENT.send_request(signal,value)
    print ("Result -> Gripper Closed.")
    RES_ABB = "null"

    # === Close GRIPPER === #

    time.sleep(0.5)

    TYPE = "LIN"
    SPEED = 0.1
    TARGET_POSE = Pose()
    TARGET_POSE.position.x = yo/1000 +0.35
    TARGET_POSE.position.y = xo/1000 + 0.2
    TARGET_POSE.position.z = 1.15
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

    rclpy.shutdown()

if __name__ == '__main__':
    main()