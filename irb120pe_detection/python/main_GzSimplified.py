#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# main_GzSimplified.py
# Main script of the Cube Detection -> ABB IRB120 Pose Estimation Use-Case:
# GAZEBO SIMULATION -> Simplified (without cube detection, which is replaced by ObjectPose plugin).

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
# Extra required libraries:
import time

# ===== IMPORT FUNCTIONS ===== #
from gripper_Gz import GzGripper
from robot import RBT

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def main(args=None):

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ABB IRB120: Pose Estimation for CUBE Pick&Place")
    print("Python script -> main_GzSimplified.py")
    print("")

    # Initialise ROS2:
    rclpy.init(args=None)

    # Initialise CLASSES:
    #GRIPPER = abbRWS_IO()
    ROBOT = RBT()
    GRIPPER = GzGripper()
    print("")

    rclpy.shutdown() 

if __name__ == '__main__':
    main()