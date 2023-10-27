#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# main.py
# Main script of the Cube Detection -> ABB IRB120 Pose Estimation Use-Case:

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
# Extra required libraries:
import time

# ===== IMPORT FUNCTIONS ===== #
from waypoints import waypoints
from gripper import abbRWS_IO

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

    # Initialise ROS2:
    rclpy.init(args=None)

    # Initialise CLASSES:
    GRIPPER = abbRWS_IO()

    # ========== (0) ========== #
    # HOME -> Open Gripper + Robot to HomePos:
    GRIPPER.OPEN()
    time.sleep(0.5)
    # Calib MoveIt!2
    # Move Robot to HomePos

    rclpy.shutdown() 

if __name__ == '__main__':
    main()