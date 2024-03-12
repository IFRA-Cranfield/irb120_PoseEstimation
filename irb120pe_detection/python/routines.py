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

# routines.py
# Python script containing the routines(executions) of the ROBOT:

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Extra required libraries:
import time
# Pose:
from geometry_msgs.msg import Pose

# ===== IMPORT FUNCTIONS ===== #
from gripper_Gz import GzGripper
from gripper import abbRWS_IO
from robot import RBT
from spawn import EEQuaternion

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

class RoutineList():

    def __init__(self, ENVIRONMENT):
        self.ROBOT = RBT()
        if (ENVIRONMENT == "GAZEBO"):
            self.GRIPPER = GzGripper()
        else: 
            self.GRIPPER = abbRWS_IO()    

    def HomePos(self):
        print("(Robot Movement -> /Move): HomePos")
        self.ROBOT.Move_EXECUTE("HomePos")

    def PickCube(self, x, y, yaw):

        print("===== ROUTINE EXECUTION =====")
        print(" - Picking cube from workspace...")
        print("")
        
        # Calculate EE ROTATION:
        EEPose = EEQuaternion(yaw)

        # Move to PickApproach:
        print("(Robot Movement -> /RobMove): PickCube_App")
        TARGET_POSE = Pose()
        TARGET_POSE.position.x = x
        TARGET_POSE.position.y = y
        TARGET_POSE.position.z = 1.10
        TARGET_POSE.orientation = EEPose.orientation
        self.ROBOT.RobMove_EXECUTE_cstm("PTP",0.3,TARGET_POSE)

        # Move to Pick:
        print("(Robot Movement -> /RobMove): PickCube")
        TARGET_POSE = Pose()
        TARGET_POSE.position.x = x
        TARGET_POSE.position.y = y
        TARGET_POSE.position.z = 1.07
        TARGET_POSE.orientation = EEPose.orientation
        self.ROBOT.RobMove_EXECUTE_cstm("LIN",0.1,TARGET_POSE)

        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)

        # Move to PickApproach:
        print("(Robot Movement -> /RobMove): PickCube_App")
        TARGET_POSE = Pose()
        TARGET_POSE.position.x = x
        TARGET_POSE.position.y = y
        TARGET_POSE.position.z = 1.10
        TARGET_POSE.orientation = EEPose.orientation
        self.ROBOT.RobMove_EXECUTE_cstm("LIN",0.1,TARGET_POSE)

    def RotateCube(self, RotBack):

        print("===== ROUTINE EXECUTION =====")
        print(" - Rotating cube 90 degrees...")
        print("")

        print("(Robot Movement -> /RobMove): RotApp")
        self.ROBOT.RobMove_EXECUTE("RotApp", "PTP", 0.3)

        if (RotBack == True):

            print("(Robot Movement -> /Move): RotBack")
            self.ROBOT.Move_EXECUTE("RotBack")


        print("(Robot Movement -> /Move): RotPlace")
        self.ROBOT.Move_EXECUTE("RotPlace")

        # OPEN GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)

        if (RotBack == True):

            print("(Robot Movement -> /Move): RotZ")
            self.ROBOT.Move_EXECUTE("RotZ")

            print("(Robot Movement -> /Move): RotBack_2")
            self.ROBOT.Move_EXECUTE("RotBack_2")

            print("(Robot Movement -> /Move): RotPlace")
            self.ROBOT.Move_EXECUTE("RotPlace")

        print("(Robot Movement -> /Move): RotationPick")
        self.ROBOT.Move_EXECUTE("RotationPick")
        print("(Robot Movement -> /Move): RotationPick")
        self.ROBOT.Move_EXECUTE("RotationPick")

        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)

        print("(Robot Movement -> /Move): RotZ")
        self.ROBOT.Move_EXECUTE("RotZ")

        print("(Robot Movement -> /Move): RotationPlace")
        self.ROBOT.Move_EXECUTE("RotationPlace")
        print("(Robot Movement -> /Move): RotationPlace")
        self.ROBOT.Move_EXECUTE("RotationPlace")

        print("(Robot Movement -> /Move): RotX")
        self.ROBOT.Move_EXECUTE("RotX")

        print("(Robot Movement -> /Move): RotationPlace")
        self.ROBOT.Move_EXECUTE("RotationPlace")

        print("(Robot Movement -> /Move): RotPlace2")
        self.ROBOT.Move_EXECUTE("RotPlace2")

        # OPEN GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)

        print("(Robot Movement -> /Move): RotZ")
        self.ROBOT.Move_EXECUTE("RotZ")
        
        print("(Robot Movement -> /Move): RotationPick")
        self.ROBOT.Move_EXECUTE("RotationPick")

        print("(Robot Movement -> /Move): RotPlace3")
        self.ROBOT.Move_EXECUTE("RotPlace3")

        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)

        print("(Robot Movement -> /Move): RotZ")
        self.ROBOT.Move_EXECUTE("RotZ")

    def PlaceCube(self, CUBE):

        if CUBE == "WhiteCube":

            print("(Robot Movement -> /RobMove): PlaceWHITE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceWHITE_app", "PTP", 0.3)

            print("(Robot Movement -> /RobMove): PlaceWHITE")
            self.ROBOT.RobMove_EXECUTE("PlaceWHITE", "LIN", 0.1)

            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)

            print("(Robot Movement -> /RobMove): PlaceWHITE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceWHITE_app", "LIN", 0.1)
            

        elif CUBE == "BlackCube":

            print("(Robot Movement -> /RobMove): PlaceBLACK_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLACK_app", "PTP", 0.3)

            print("(Robot Movement -> /RobMove): PlaceBLACK")
            self.ROBOT.RobMove_EXECUTE("PlaceBLACK", "LIN", 0.1)

            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)

            print("(Robot Movement -> /RobMove): PlaceBLACK_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLACK_app", "LIN", 0.1)

        elif CUBE == "BlueCube":

            print("(Robot Movement -> /RobMove): PlaceBLUE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLUE_app", "PTP", 0.3)

            print("(Robot Movement -> /RobMove): PlaceBLUE")
            self.ROBOT.RobMove_EXECUTE("PlaceBLUE", "LIN", 0.1)

            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)

            print("(Robot Movement -> /RobMove): PlaceBLUE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLUE_app", "LIN", 0.1)

        elif CUBE == "Cube":
            
            print("(Robot Movement -> /RobMove): PlaceCUBE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceCUBE_app", "PTP", 0.3)

            print("(Robot Movement -> /RobMove): PlaceCUBE")
            self.ROBOT.RobMove_EXECUTE("PlaceCUBE", "LIN", 0.1)

            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)

            print("(Robot Movement -> /RobMove): PlaceCUBE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceCUBE_app", "LIN", 0.1)

    def CubeMid(self):

        print("(Robot Movement -> /RobMove): PlaceMidApp")
        self.ROBOT.RobMove_EXECUTE("PlaceMidApp", "PTP", 0.3)

        print("(Robot Movement -> /RobMove): PlaceMid")
        self.ROBOT.RobMove_EXECUTE("PlaceMid", "LIN", 0.1)

        # OPEN GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)
    
    def CheckTop(self, DETECTION):

        print("===== ROUTINE EXECUTION =====")
        print(" - Checking colour of cube's TOP face...")
        print("")

        self.CubeMid()

        print("(Robot Movement -> /Move): PickTop")
        self.ROBOT.Move_EXECUTE("PickTop")

        print("(Robot Movement -> /Move): PickTop")
        self.ROBOT.Move_EXECUTE("PickTop")

        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)

        print("(Robot Movement -> /Move): PickTopApp")
        self.ROBOT.Move_EXECUTE("PickTopApp")

        print("(Robot Movement -> /RobMove): FacePose_4")
        self.ROBOT.RobMove_EXECUTE("FacePose_4", "PTP", 0.3)

        COLOUR = DETECTION.DetectColour()
        #DETECTION.TestDetection()

        print("(Robot Movement -> /RobMove): RePickTopApp_PREV")
        self.ROBOT.RobMove_EXECUTE("RePickTopApp_PREV", "PTP", 0.3)

        print("(Robot Movement -> /Move): RePickTop_PREV")
        self.ROBOT.Move_EXECUTE("RePickTop_PREV")

        # OPEN GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)

        print("(Robot Movement -> /Move): RePickTop")
        self.ROBOT.Move_EXECUTE("RePickTop")

        print("(Robot Movement -> /Move): RePickTop")
        self.ROBOT.Move_EXECUTE("RePickTop")

        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)

        print("(Robot Movement -> /Move): RePickTopApp")
        self.ROBOT.Move_EXECUTE("RePickTopApp")

        return(COLOUR)
    
    def CheckCube(self, DETECTION):

        # Define RESULT variable:
        RESULT = {"COLOUR": "Cube", "SIDE": "None"}

        print("===== ROUTINE EXECUTION =====")
        print(" - Checking sticker location and feature colour...")
        print("")

        print("(Robot Movement -> /RobMove): FacePose_1")
        self.ROBOT.RobMove_EXECUTE("FacePose_1", "PTP", 0.3)

        COLOUR = DETECTION.DetectColour()

        if COLOUR != "Cube":
            RESULT["COLOUR"] = COLOUR
            RESULT["SIDE"] = "FRONT"
            return(RESULT)
        
        print("(Robot Movement -> /Move): FacePose_2")
        self.ROBOT.Move_EXECUTE("FacePose_2")

        COLOUR = DETECTION.DetectColour()

        if COLOUR != "Cube":
            RESULT["COLOUR"] = COLOUR
            RESULT["SIDE"] = "BACK"
            return(RESULT)
        
        print("(Robot Movement -> /RobMove): FacePose_3")
        self.ROBOT.RobMove_EXECUTE("FacePose_3", "PTP", 0.3)

        COLOUR = DETECTION.DetectColour()

        print("(Robot Movement -> /RobMove): FacePose_1")
        self.ROBOT.RobMove_EXECUTE("FacePose_1", "PTP", 0.3)

        if COLOUR != "Cube":
            RESULT["COLOUR"] = COLOUR
            RESULT["SIDE"] = "BOTTOM"
            return(RESULT)

        return(RESULT)
    
    def CheckCubeSides(self, DETECTION):

        self.CubeMid()

        print("(Robot Movement -> /RobMove): PlaceMidApp")
        self.ROBOT.RobMove_EXECUTE("PlaceMidApp", "LIN", 0.3)

        print("(Robot Movement -> /Move): PickSideApp_1")
        self.ROBOT.Move_EXECUTE("PickSideApp_1")

        print("(Robot Movement -> /Move): PickSide")
        self.ROBOT.Move_EXECUTE("PickSide")

        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)

        print("(Robot Movement -> /Move): PickSideApp_2")
        self.ROBOT.Move_EXECUTE("PickSideApp_2")

        CheckRES = self.CheckCube(DETECTION)

        return(CheckRES)

