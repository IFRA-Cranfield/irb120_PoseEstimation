#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# routines.py
# Python script containing the routines(executions) of the ROBOT:

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Extra required libraries:
import time
# Pose:
from geometry_msgs.msg import Pose

# ===== IMPORT FUNCTIONS ===== #
from gripper_Gz import GzGripper
from robot import RBT
from spawn import EEQuaternion

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

class RoutineList():

    def __init__(self):
        self.ROBOT = RBT()
        self.GRIPPER = GzGripper()

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

    def RotateCube(self):

        print("===== ROUTINE EXECUTION =====")
        print(" - Rotating cube 90 degrees...")
        print("")

        print("(Robot Movement -> /RobMove): RotApp")
        self.ROBOT.RobMove_EXECUTE("RotApp", "PTP", 0.3)

        print("(Robot Movement -> /Move): RotPlace")
        self.ROBOT.Move_EXECUTE("RotPlace")

        # OPEN GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)

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