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

# main.py
# Main script of the Cube Detection -> ABB IRB120 Pose Estimation Use-Case:

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
# Extra required libraries:
import time

# ===== IMPORT FUNCTIONS ===== #
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
    print("Python script -> main.py")
    print("")

    # Initialise ROS2:
    rclpy.init(args=None)

    # Initialise CLASSES:
    ROUTINE = RoutineList("ROBOT")
    print("")

    # Move ROBOT to HomePos:
    ROUTINE.HomePos()

    # Initalise DETECTION class (+ CAMERA INPUT):
    DETECTION = CubeDetection("ROBOT")

    # CALIBRATION w/ ARUCOS -> Get PERSPECTIVE IMG:
    DETECTION.GetPerspectiveImg(ShowPerspective=True)

    # Obtain CUBE LOCATION:
    DetectRES = DETECTION.CubeLocation()

    if not DetectRES["success"]:
        
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

    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()