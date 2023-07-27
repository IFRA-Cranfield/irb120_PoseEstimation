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
from irb120pe_data.srv import Calibrate
from irb120pe_data.srv import Estimate
from irb120pe_data.action import Move
from sensor_msgs.msg import Image

# Required to use OpenCV:
import cv2
import os
from cv_bridge import CvBridge

# Data classes:
from dataclasses import dataclass

# Define GLOBAL VARIABLE:
PERSPECTIVE = Image()

# Define GLOBAL VARIABLE -> RES:
@dataclass
class feedback:
    MESSAGE: String
    SUCCESS: bool
RES = feedback("null", False)

# Define GLOBAL VARIABLE -> RES_PE:
@dataclass
class PE_Result:
    x: float
    y: float
    rotation: float
RES_PE = PE_Result(0.0, 0.0, 0.0)

# Required to include YOLOv8 module:
# TBD.

# =============================================================================== #
# ROS2 ActionClient for RobMove:

class MoveClient(Node):

    def __init__(self):

        super().__init__('irb120pe_RobMove_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        print ("Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print ("/Move ACTION SERVER detected.")

    def send_goal(self, TYPE, SPEED, TARGET_POSE):
        
        # 1. Assign variables:
        goal_msg = Move.Goal()
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
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
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
        RES.MESSAGE = result.message
        RES.SUCCESS = result.success
        
        # 2. Print RESULT:
        print ("RobMove ACTION CALL finished.") 
        print ("MESSAGE: " + RES.MESSAGE)
        print ("SUCCESS: " + str(RES.SUCCESS))

# =============================================================================== #
# ROS2 ServiceClient for Calibration:

class CalibrateClient(Node):

    def __init__(self):

        super().__init__('irb120pe_Calibration_Client')                                                  
        self.cli = self.create_client(Calibrate, "CALIBRATE")     

        while not self.cli.wait_for_service(timeout_sec=1.0):                                      
            print ("Waiting for /CALIBRATE ROS2 ServiceServer to be available...")                                                                                                             
        print ("/CALIBRATE SERVICE SERVER detected.")

        self.req = Calibrate.Request() 

    def CALIBRATE_request(self, FRAME):

        global RES
        global PERSPECTIVE

        # Assign values to the variables in the .srv -> request:
        self.req.frame = FRAME

        # Call ROS2 Service ONCE, and wait until it is finished:
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)  

        # Assign RESULT:
        RESULT = self.future.result() 
        RES.MESSAGE = RESULT.message
        RES.SUCCESS = RESULT.success  
        PERSPECTIVE = RESULT.perspective

# =============================================================================== #
# ROS2 ServiceClient for PoseEstimation:

class EstimateClient(Node):

    def __init__(self):

        super().__init__('irb120pe_PoseEstimation_Client')                                                  
        self.cli = self.create_client(Estimate, "ESTIMATE")     

        while not self.cli.wait_for_service(timeout_sec=1.0):                                      
            print ("Waiting for /ESTIMATE ROS2 ServiceServer to be available...")                                                                                                             
        print ("/ESTIMATE SERVICE SERVER detected.")

        self.req = Estimate.Request() 

    def ESTIMATE_request(self, ROI):

        global RES
        global RES_PE

        # Assign values to the variables in the .srv -> request:
        self.req.roi = ROI

        # Call ROS2 Service ONCE, and wait until it is finished:
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)  
        
        # Assign RES + RES_PE:
        RESULT = self.future.result() 
        RES_PE.x = RESULT.x
        RES_PE.y = RESULT.y
        RES_PE.rotation = RESULT.rotation
        RES.MESSAGE = RESULT.message
        RES.SUCCESS = RESULT.success 

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
    global RES_PE
    global PERSPECTIVE

    # 1. INITIALISE ROS NODE:
    rclpy.init(args=args)

    # 2. INITIALISE ACTION/SERVICE CLIENTS:
    #RobMove_CLIENT = MoveClient()
    Calibration_CLIENT = CalibrateClient()
    #PoseEstimation_CLIENT = EstimateClient()

    # 3. TAKE FRAME FROM CAMERA:
    camera = cv2.VideoCapture(0) #Define the camera and its port
    ret, image = camera.read()
    if not ret:
        print("Error! Failed at capturing the image")
    camera.release()

    # 4. CONVERT FRAME img (in OpenCV format) to sensor_msgs/Image:
    bridge = CvBridge()

    # === Call Calibration SERVICE === #

    FRAME = bridge.cv2_to_imgmsg(image, 'bgr8')
    Calibration_CLIENT.CALIBRATE_request(FRAME)

    print("RESULT of Calibration SERVICE CALL: " + RES.MESSAGE)
    print("SERVICE CALL successful? -> " + str(RES.SUCCESS))
    print("")
    
    RES.MESSAGE = "null"
    RES.SUCCESS = False

    PERSPECTIVE_cv = bridge.imgmsg_to_cv2(PERSPECTIVE, 'bgr8')
    cv2.imshow("Perspective Image", PERSPECTIVE_cv)
    cv2.waitKey(0)

    # === Call Calibration SERVICE === #

    # === Call PoseEstimation SERVICE === #

    #ROI = ()
    #PoseEstimation_CLIENT.ESTIMATE_request(ROI)

    #PoseEstimation_RESULT = RES_PE
    
    #print("RESULT of PoseEstimation SERVICE CALL: " + RES.MESSAGE)
    #print("SERVICE CALL successful? -> " + RES.SUCCESS)
    #print("")
    
    #RES_PE.x = 0.0
    #RES_PE.y = 0.0
    #RES_PE.rotation = 0.0
    #RES.MESSAGE = "null"
    #RES.SUCCESS = False

    # === Call PoseEstimation SERVICE === #
    
    # === Call RobMove ACTION === #
    
    #TYPE = "PTP" // "LIN"
    #SPEED = 0.0
    #TARGET_POSE = Pose()
    #TARGET_POSE.position.x = 0.0
    #TARGET_POSE.position.y = 0.0
    #TARGET_POSE.position.z = 0.0
    #TARGET_POSE.orientation.x = 0.0
    #TARGET_POSE.orientation.y = 0.0
    #TARGET_POSE.orientation.z = 0.0
    #TARGET_POSE.orientation.w = 0.0

    #RobMove_CLIENT.send_goal(TYPE, SPEED, TARGET_POSE)
    
    #while rclpy.ok():
    #    rclpy.spin_once(RobMove_CLIENT)
    #    if (RES != "null"):
    #        break
    
    #print("RESULT of RobMove ACTION CALL: " + RES.MESSAGE)
    #print("ACTION CALL successful? -> " + RES.SUCCESS)
    #print("")

    #RES.MESSAGE = "null"
    #RES.SUCCESS = False

    # === Call RobMove ACTION === #

    rclpy.shutdown()

if __name__ == '__main__':
    main()