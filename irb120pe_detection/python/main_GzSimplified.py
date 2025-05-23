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

# main_GzSimplified.py
# Main script of the Cube Detection -> ABB IRB120 Pose Estimation Use-Case:
# GAZEBO SIMULATION -> Simplified (without cube detection, which is replaced by ObjectPose plugin).

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
    SPAWN = SpawnCube()
    DELETE = DeleteCube()
    ROUTINE = RoutineList()
    print("")

    # Move ROBOT to HomePos:
    ROUTINE.HomePos()

    # SPAWN random cube:
    OPT_cube = ["WhiteCube","BlackCube","BlueCube","Cube"]
    VAR_cube = random.choice(OPT_cube)
    OPT_or = ["TOP","BOTTOM","FRONT"]
    VAR_or = random.choice(OPT_or)
    print("Spawning random CUBE at a randome POSE to the Gazebo Environment...")
    SpawnRES = SPAWN.SPAWN(VAR_cube,VAR_or)

    if (SpawnRES["success"] == False):
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()  

    # PICK cube:
    ROUTINE.PickCube(SpawnRES["x"],SpawnRES["y"],SpawnRES["yaw"])

    # ROTATE if -> FRONT/BOTTOM:
    if (VAR_cube != "Cube"):

        if (VAR_or == "FRONT"):
            ROUTINE.RotateCube(RotBack = False)
        elif (VAR_or == "BOTTOM"):
            ROUTINE.RotateCube(RotBack = False)
            ROUTINE.RotateCube(RotBack = False)

    # PLACE CUBE:
    ROUTINE.PlaceCube(VAR_cube)

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