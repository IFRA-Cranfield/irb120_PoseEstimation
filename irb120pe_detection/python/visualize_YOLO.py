#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# visualize_YOLO.py
# Script used to constantly OUTPUT the YOLOv8 model detection:

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node

# ===== IMPORT FUNCTIONS ===== #
from detection import CubeDetection

# ===== INPUT PARAMETERS ===== #
# Environment (Gazebo/Robot):

PARAM_ENV = "default"
P_CHECK_ENV = False

class envPARAM(Node):

    def __init__(self):

        global PARAM_ENV
        global P_CHECK_ENV
        
        super().__init__('irb120pe_vYOLO_envPARAM')
        self.declare_parameter('ENVIRONMENT', "default")
        PARAM_ENV = self.get_parameter('ENVIRONMENT').get_parameter_value().string_value
        
        if (PARAM_ENV == "default"):

            print('ENVIRONMENT ROS2 Parameter was not defined.')
            print('COMMAND -> ros2 run irb120pe_detection visualize_YOLO.py --ros-args -p ENVIRONMENT:="---"')
            print("")
            
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

        else:

            print('ENVIRONMENT ROS2 Parameter received: ' + PARAM_ENV)
            
            if (PARAM_ENV == "GAZEBO" or PARAM_ENV == "ROBOT"):
                None 
                
            else:
                
                print('ERROR: Environment not well defined -> It must be GAZEBO or ROBOT.')
                print('COMMAND -> ros2 run irb120pe_detection visualize_YOLO.py --ros-args -p ENVIRONMENT:="---"')
                print("")
                
                rclpy.shutdown()
                print("CLOSING PROGRAM...")
                exit()
        
        P_CHECK_ENV = True

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def main(args=None):

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ABB IRB120: Pose Estimation for CUBE Pick&Place")
    print("Python script -> visualize_YOLO.py")
    print("")

    rclpy.init()

    global PARAM_ENV
    global P_CHECK_ENV

    envNODE = envPARAM()
    while (P_CHECK_ENV == False):
        rclpy.spin_once(envNODE)
    envNODE.destroy_node()

    DETECTION = CubeDetection(PARAM_ENV)
    DETECTION.ConstantVisualization()

    print("")
    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")

    rclpy.shutdown() 

if __name__ == '__main__':
    main()