#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso         - Mikel.Bueno-Viso@cranfield.ac.uk                 #
#           Irene Bernardino Sanchez - i.bernardinosanchez.854@cranfield.ac.uk          #
#           Seemal Asif              - s.asif@cranfield.ac.uk                           #
#           Phil Webb                - p.f.webb@cranfield.ac.uk                         #
#                                                                                       #
#  Date: November, 2023.                                                                #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023). Object Detection and Pose Estimation within a Robot Cell. URL: https://github.com/IFRA-Cranfield/irb120_PoseEstimation

# gripper_Gz.py
# This function opens/closes the Schunk EGP-64 gripper in the Gazebo Simulation, and activates the IFRA_LinkAttacher plugin if the gripper is about to grasp any object in the workspace.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
# Import /Move and /RobMove ROS2 Actions:
from ros2srrc_data.action import Move
# Import LinkAttacher (ROS2 SRV):
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
# Import ROS2 messages:
from std_msgs.msg import String
from ros2srrc_data.msg import Action
from objectpose_msgs.msg import ObjectPose
from linkpose_msgs.msg import LinkPose

# GLOBAL VARIABLE -> CUBES:
CUBES = {}
CUBES["WhiteCube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}
CUBES["BlackCube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}
CUBES["BlueCube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}
CUBES["Cube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}

# GLOBAL VARIABLE -> AttachCheck:
from dataclasses import dataclass
@dataclass
class AttDetCHECK:
    ATTACHED: bool
    MODEL: String
    LINK: String
AttachCheck = AttDetCHECK(False, "", "")

# GLOBAL VARIABLE -> RES:
@dataclass
class RobotRES:
    MESSAGE: String
    SUCCESS: bool
RES = RobotRES("null", False)

# GLOBAL VARIABLE -> EE_POSE:
EE_POSE = LinkPose()

# Dictionary for TESTING/RESULTS:
TESTING = {"EEMoveOK": 0, "EEMoveNoOK": 0}

# =============================================================================== #
# /Move ACTION CLIENT:

class MoveCLIENT(Node):

    def __init__(self):

        super().__init__('GzGripper_Move_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        print("(/Move)-Gripper: Initialising ROS2 Action Client!")
        print("(/Move)-Gripper: Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("(/Move)-Gripper: /Move ACTION SERVER detected.")

    def send_goal(self, ACTION):

        goal_msg = Move.Goal()
        goal_msg.action = ACTION.action
        goal_msg.speed = ACTION.speed
        goal_msg.moveg = ACTION.moveg
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('(/Move)-Gripper: Move ACTION CALL -> GOAL has been REJECTED.')
            return
        
        # print('(/Move): Move ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        RESULT = future.result().result
        RES.MESSAGE = RESULT.result

        if "FAILED" in RES.MESSAGE:
            RES.SUCCESS = False
        else:
            RES.SUCCESS = True

# =============================================================================== #
# CubePose SUBSCRIBER:

class CubePose(Node):

    def __init__(self):

        super().__init__("irb120pe_CubePose_Subscriber")
        self.SUBWhite = self.create_subscription(ObjectPose, "/WhiteCube/ObjectPose", self.CALLBACK_FN, 10)
        self.SUBBlack = self.create_subscription(ObjectPose, "/BlackCube/ObjectPose", self.CALLBACK_FN, 10)
        self.SUBBlue = self.create_subscription(ObjectPose, "/BlueCube/ObjectPose", self.CALLBACK_FN, 10)
        self.SUBCube = self.create_subscription(ObjectPose, "/Cube/ObjectPose", self.CALLBACK_FN, 10)

    def CALLBACK_FN(self, OBJ):

        global CUBES

        for Cube, Pose in CUBES.items():

            if (OBJ.objectname == Cube):

                Pose["x"] = OBJ.x
                Pose["y"] = OBJ.y
                Pose["z"] = OBJ.z
                Pose["qx"] = OBJ.qx
                Pose["qy"] = OBJ.qy
                Pose["qz"] = OBJ.qz
                Pose["qw"] = OBJ.qw

# =============================================================================== #
# EEPose SUBSCRIBER:

class EEPose(Node):

    def __init__(self):

        super().__init__("irb120pe_EEPose_Subscriber")
        self.SUB_EE = self.create_subscription(LinkPose, "/LinkPose_irb120_EE_egp64", self.CALLBACK_FN, 10)

    def CALLBACK_FN(self, EE):

        global EE_POSE
        EE_POSE = EE

# =============================================================================== #
# LinkAttacher SERVICE CLIENT:

class LinkAttacher_Client(Node):

    def __init__(self):

        super().__init__("irb120pe_LinkAttacher_Client")

        self.AttachClient = self.create_client(AttachLink, "/ATTACHLINK")
        self.DetachClient = self.create_client(DetachLink, "/DETACHLINK")

        while not self.AttachClient.wait_for_service(timeout_sec=1.0): 
            print("(LinkAttacher): /ATTACHLINK ROS2 Service not still available, waiting...")
        print("(LinkAttacher): /ATTACHLINK ROS2 Service ready.")
        while not self.DetachClient.wait_for_service(timeout_sec=1.0): 
            print("(LinkAttacher): /DETACHLINK ROS2 Service not still available, waiting...")
        print("(LinkAttacher): /DETACHLINK ROS2 Service ready.")

        self.AttachRequest = AttachLink.Request()
        self.DetachRequest = DetachLink.Request()

    def ATTACHService(self, MODEL, LINK):

        self.AttachRequest.model1_name = "irb120"
        self.AttachRequest.link1_name = "EE_egp64"
        self.AttachRequest.model2_name = MODEL
        self.AttachRequest.link2_name = LINK

        self.AttachFuture = self.AttachClient.call_async(self.AttachRequest)

    def DETACHService(self, MODEL, LINK):

        self.DetachRequest.model1_name = "irb120"
        self.DetachRequest.link1_name = "EE_egp64"
        self.DetachRequest.model2_name = MODEL
        self.DetachRequest.link2_name = LINK

        self.DetachFuture = self.DetachClient.call_async(self.DetachRequest)

class LinkAttacher():

    def __init__(self):
        self.CLIENT = LinkAttacher_Client()

    def ATTACH(self, MODEL, LINK):

        global AttachCheck

        self.CLIENT.ATTACHService(MODEL,LINK)

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.AttachFuture.done():
                try:
                    AttachRES = self.CLIENT.AttachFuture.result()
                except Exception as exc:
                    print("(LinkAttacher): /ATTACHLINK Service call failed -> " + str(exc))
                    return(False)
                else:
                    if (AttachRES.success):
                        print("(LinkAttacher): /ATTACHLINK successful -> " + str(AttachRES.message))
                        AttachCheck.ATTACHED = True
                        AttachCheck.MODEL = MODEL
                        AttachCheck.LINK = LINK
                        return(True)
                    else:
                        print("(LinkAttacher): /ATTACHLINK unuccessful -> " + str(AttachRES.message))
                        return(False)
                    
    def DETACH(self, MODEL, LINK):

        global AttachCheck

        self.CLIENT.DETACHService(MODEL,LINK)

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.DetachFuture.done():
                try:
                    DetachRES = self.CLIENT.DetachFuture.result()
                except Exception as exc:
                    print("(LinkAttacher): /DETACHLINK Service call failed -> " + str(exc))
                    return(False)
                else:
                    if (DetachRES.success):
                        print("(LinkAttacher): /DETACHLINK successful -> " + str(DetachRES.message))
                        AttachCheck.ATTACHED = False
                        AttachCheck.MODEL = ""
                        AttachCheck.LINK = ""
                        return(True)
                    else:
                        print("(LinkAttacher): /DETACHLINK unuccessful -> " + str(DetachRES.message))
                        return(False)

# =============================================================================== #
# GzGripper class, to OPEN/CLOSE the Gripper in Gazebo:

class GzGripper():

    def __init__(self):

        # Initialise /Move Action Client:
        self.MoveClient = MoveCLIENT()

        # Initialise EEPose and CubePose classes:
        self.CubeLocation = CubePose()
        self.EEPose = EEPose()

        # Initialise LinkAttacher class:
        self.LinkAttacher = LinkAttacher()

    def CLOSE(self):

        # Close GRIPPER -> /Move:
        
        WP = Action()
        WP.action = "MoveG"
        WP.speed = 1.0
        WP.moveg = 0.008

        self.MoveClient.send_goal(WP)

        while rclpy.ok():

            rclpy.spin_once(self.MoveClient)

            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == True):
            
            print("Result -> " + RES.MESSAGE)
            
            TESTING["EEMoveOK"] = TESTING["EEMoveOK"] + 1
            print(TESTING)
            print()
            
            RES.MESSAGE = "null"
            RES.SUCCESS = False

        elif (RES.SUCCESS == False):
    
            print("Result -> " + RES.MESSAGE)
            
            TESTING["EEMoveNoOK"] = TESTING["EEMoveNoOK"] + 1
            print(TESTING)
            print("")

            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

        # ===== CHECK GRASPING ===== #

        global CUBES
        global EE_POSE

        # Loop for 0.1 seconds in order to get EE and CUBE poses:
        t_end = time.time() + 0.1
        while time.time() < t_end:
            rclpy.spin_once(self.CubeLocation)
            rclpy.spin_once(self.EEPose)

        # 1. Check which CUBE is in workspace, and get its POSE:
        OBJ = ObjectPose()
        Found = False
        for Cube, Pose in CUBES.items():

            for VALUE in Pose.values():

                if VALUE != 0.0:
                    Found = True
                    break

            if Found:
                OBJ.objectname = Cube
                OBJ.x = Pose["x"]
                OBJ.y = Pose["y"]
                OBJ.z = Pose["z"]
                OBJ.qx = Pose["qx"]
                OBJ.qy = Pose["qy"]
                OBJ.qz = Pose["qz"]
                OBJ.qw = Pose["qw"]
                break

        if not Found:
            print("(GzGripper): Gripper closed without grasping any object.")
            print("")
            return()
        
        # 2. Check if CubePose and EEPose match:
        CheckATT = True
        if (EE_POSE.x - 0.01 > OBJ.x) or (EE_POSE.x + 0.01 < OBJ.x):
            CheckATT = False
        if (EE_POSE.y - 0.01 > OBJ.y) or (EE_POSE.y + 0.01 < OBJ.y):
            CheckATT = False
        if (EE_POSE.z - 0.01 > OBJ.z) or (EE_POSE.z + 0.01 < OBJ.z):
            CheckATT = False

        # 3. ATTACH if Check=True:
        if CheckATT:
            
            # LinkAttacher:
            AttRES = self.LinkAttacher.ATTACH(OBJ.objectname, OBJ.objectname)
            if AttRES:
                print("(GzGripper): Gripper closed, object->" + str(OBJ.objectname) + " attached.")
                print("")
            else: 
                print("(GzGripper): Gripper closed, object->" + str(OBJ.objectname) + " not attached, LinkAttacher did not work properly.")
                print("")

        else:
            print("(GzGripper): Gripper closed, object->" + str(OBJ.objectname) + " not attached, gripper and object poses don't match.")
            print("")

    def OPEN(self):

        # Open GRIPPER -> /Move:
        
        WP = Action()
        WP.action = "MoveG"
        WP.speed = 1.0
        WP.moveg = 0.0

        self.MoveClient.send_goal(WP)

        while rclpy.ok():

            rclpy.spin_once(self.MoveClient)

            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == True):
            
            print("Result -> " + RES.MESSAGE)
            
            TESTING["EEMoveOK"] = TESTING["EEMoveOK"] + 1
            print(TESTING)
            print()
            
            RES.MESSAGE = "null"
            RES.SUCCESS = False

        elif (RES.SUCCESS == False):
    
            print("Result -> " + RES.MESSAGE)
            
            TESTING["EEMoveNoOK"] = TESTING["EEMoveNoOK"] + 1
            print(TESTING)
            print("")

            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

        # CHECK if --> There is any object currently grasped:
        global AttachCheck 
        MODELname = AttachCheck.MODEL

        if AttachCheck.ATTACHED:

            DetRES = self.LinkAttacher.DETACH(AttachCheck.MODEL, AttachCheck.LINK)

            if DetRES:
                print("(GzGripper): Gripper opened, object->" + str(MODELname) + " detached.")
                print("")
            else: 
                print("(GzGripper): Gripper opened, object->" + str(MODELname) + " not detached, LinkAttacher did not work properly.")
                print("")

        else:

            print("(GzGripper): Gripper opened without dropping any object.")
            print("")