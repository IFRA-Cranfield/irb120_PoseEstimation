#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# main_Gz.py
# Main script of the Cube Detection -> ABB IRB120 Pose Estimation Use-Case:
# GAZEBO SIMULATION -> With cube detection using the Gz CAMERA PLUGIN.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
# Extra required libraries:
import time
import random

# ===== IMPORT FUNCTIONS ===== #
from spawn import SpawnCube
from spawn import DeleteCube
from routines import RoutineList
from detection import CubeDetection

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def main(args=None):

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ABB IRB120: Pose Estimation for CUBE Pick&Place")
    print("Python script -> main_Gz.py")
    print("")

    # Initialise ROS2:
    rclpy.init(args=None)

    # Initialise CLASSES:
    SPAWN = SpawnCube()
    DELETE = DeleteCube()
    ROUTINE = RoutineList()
    print("")

    # Move ROBOT to HomePos:
    ROUTINE.HomePos()

    # SPAWN random cube:
    OPT_cube = ["WhiteCube","BlackCube","BlueCube","Cube"]
    VAR_cube = random.choice(OPT_cube)
    print("Spawning random CUBE at a randome POSE to the Gazebo Environment...")
    SpawnRES = SPAWN.SPAWN(VAR_cube,"RANDOM")

    if (SpawnRES["success"] == False):
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()  

    # Initalise DETECTION class (+ CAMERA INPUT):
    DETECTION = CubeDetection("GAZEBO")

    # CALIBRATION w/ ARUCOS -> Get PERSPECTIVE IMG:
    DETECTION.GetPerspectiveImg(ShowPerspective=True)

    # Test YOLOv8 detection (UNCOMMENT THIS LINE TO TEST IF THE YOLOv8 MODEL IS DETECTING THE CUBE IN GAZEBO):
    # DETECTION.TestDetection()

    # Obtain CUBE LOCATION:
    DetectRES = DETECTION.CubeLocation()

    if not DetectRES["success"]:
        
        DELETE.DELETE(VAR_cube)
        
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()

    # PICK cube:
    ROUTINE.PickCube(DetectRES["x"],DetectRES["y"],DetectRES["yaw"])

    # INIT -> CubeSide and CubeColour variables:
    CubeSide = ""
    CubeColour = DetectRES["detection"]
    
    # Check colour -> TOP FACE:
    if (CubeColour == "Sticker"):

        CubeColour = ROUTINE.CheckTop(DETECTION)
        print(CubeColour)

    # Check colour + face (BOTTOM/FRONT/BACK/SIDE):
    if (CubeColour == "Cube"):

        CheckRES = ROUTINE.CheckCube(DETECTION)
        CubeColour = CheckRES["COLOUR"]
        CubeSide = CheckRES["SIDE"]

        if (CubeColour == "Cube"):

            CheckRES = ROUTINE.CheckCubeSides(DETECTION)
            CubeColour = CheckRES["COLOUR"]
            CubeSide = CheckRES["SIDE"] 

    # ROTATE if REQUIRED:
    if (CubeColour != "Cube"):

        if (CubeSide == "FRONT"):
            ROUTINE.RotateCube(RotBack = False)

        if (CubeSide == "BOTTOM"):
            ROUTINE.RotateCube(RotBack = False)
            ROUTINE.RotateCube(RotBack = False)

        if (CubeSide == "BACK"):
            ROUTINE.RotateCube(RotBack = True)

    # PLACE CUBE:
    ROUTINE.PlaceCube(CubeColour)

    # Finish -> Back to HomePos:
    ROUTINE.HomePos()

    time.sleep(3.0)

    # Remove CUBE from workspace, to enable another execution after this (there can only be one cube at a time on top of the workspace for the program to work):
    DELETE.DELETE(VAR_cube)

    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")

    rclpy.shutdown() 

if __name__ == '__main__':
    main()