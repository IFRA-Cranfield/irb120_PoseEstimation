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

# samples.py
# This script takes the pictures required to train the YOLOv8 model.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
# CAMERA ROS2msg:
from sensor_msgs.msg import Image
# OpenCV:
import cv2
# ROS2 to OpenCV -> cv_bridge:
from cv_bridge import CvBridge, CvBridgeError
# Spawn and delete cube in GZ:
from spawn import SpawnCube
from spawn import DeleteCube
# Extra:
import time
import os
from routines import RoutineList

# GLOBAL VARIABLE:
Gz_CAM = None

# =============================================================================== #
# CLASS -> ImgSUB:

class ImgSUB(Node):

    def __init__(self):

        super().__init__("irb120pe_GzCAM_Subscriber")
        self.SubIMAGE = self.create_subscription(Image, "/camera/image_raw", self.CALLBACK_FN, 10)
        self.BRIDGE = CvBridge()

    def CALLBACK_FN(self, ROS2img):

        global Gz_CAM

        try:
            Gz_CAM = self.BRIDGE.imgmsg_to_cv2(ROS2img, "bgr8")
        except CvBridgeError as ERR:
            print("(cv_bridge): ERROR -> " + ERR) 
            print("")


# =============================================================================== #
# CLASS -> GetSamples:

class GetSamples():

    def __init__(self, ENV):
        
        # Initialise CAMERA:
        if (ENV == "GAZEBO"):
            self.GzCAM_SUB = ImgSUB()
            self.InitCamGz()
        else:
            self.InitCam()

    def InitCam(self):
        
        # Initialise CAMERA:
        self.camera = cv2.VideoCapture(0)
        
        # Initialise RET(OpenCV variable) and inputImg(OpenCV MAT) values:
        while True:
            self.ret, self.inputImg = self.camera.read()
            cv2.imshow("IRB-120 PoseEstimation: Input Image", self.inputImg)
            key = cv2.waitKey(1)
            if key == ord('q') or self.inputImg is None:
                break 

        if self.inputImg is None:
            print("Error! Input image is empty. Please check the camera and the VideoCapture(i) index.")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()
        
        if not self.ret:
            print("Error! Failed to capture input image. Please check the camera and the VideoCapture(i) index.")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

    def InitCamGz(self):
        
        global Gz_CAM

        print("=== Gazebo CAM: Initialization ===")
        print("Loading GAZEBO CAMERA...")
        print("")

        # Initialise RET(OpenCV variable) and inputImg(OpenCV MAT) values:
        while True:

            # 1. SPIN /Image topic subscriber!
            rclpy.spin_once(self.GzCAM_SUB)

            # 2. ASSIGN + Show IMG:
            self.inputImg = Gz_CAM
            
            if self.inputImg is not None:
                self.inputImgSHOW = cv2.resize(self.inputImg, (900, 500))
                cv2.imshow("IRB-120 PoseEstimation: Input Image", self.inputImgSHOW)

            key = cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyWindow("IRB-120 PoseEstimation: Input Image")
                break 
        
        if self.inputImg is None:
            print("Error! Input image is empty. Please check that the Gz WEBCAM is publishing the IMG correctly.")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

    def SavePicture(self, FileName):

        global GZ_CAM

        PATH = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'samples')
        imgPATH = PATH + "/" + FileName

        WRITE = False

        T = time.time() + 0.1
        while time.time() < T:
            rclpy.spin_once(self.GzCAM_SUB)
            self.inputImg = Gz_CAM

        cv2.imwrite(imgPATH,self.inputImg)

        print("(cv2.imwrite): Successfully saved -> " + imgPATH)
        print("")

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def main(args=None):

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ABB IRB120: Pose Estimation for CUBE Pick&Place")
    print("Python script -> samples.py")
    print("")

    # Initialise ROS2:
    rclpy.init(args=None)

    # ===== GET SAMPLES from GAZEBO ENVIRONMENT ===== #
    GET_SAMPLES = GetSamples("GAZEBO")
    SPAWN_CUBE = SpawnCube()
    DELETE_CUBE = DeleteCube()
    ROUTINE = RoutineList()

    # START -> HomePos():
    ROUTINE.HomePos()

    # BLACK CUBE 100 times:
    for i in range (1,101):
        SPAWN_CUBE.SPAWN("BlackCube","TOP")
        FileName = "Black_" + str(i) + ".jpg"
        GET_SAMPLES.SavePicture(FileName)
        DELETE_CUBE.DELETE("BlackCube")

    # WHITE CUBE 100 times:
    for i in range (1,101):
        SPAWN_CUBE.SPAWN("WhiteCube","TOP")
        FileName = "White_" + str(i) + ".jpg"
        GET_SAMPLES.SavePicture(FileName)
        DELETE_CUBE.DELETE("WhiteCube")

    # BLUE CUBE 100 times:
    for i in range (1,101):
        SPAWN_CUBE.SPAWN("BlueCube","TOP")
        FileName = "Blue_" + str(i) + ".jpg"
        GET_SAMPLES.SavePicture(FileName)
        DELETE_CUBE.DELETE("BlueCube")

    # CUBE 100 times:
    for i in range (1,101):
        SPAWN_CUBE.SPAWN("Cube","TOP")
        FileName = "Cube_" + str(i) + ".jpg"
        GET_SAMPLES.SavePicture(FileName)
        DELETE_CUBE.DELETE("Cube")

    # Close program:
    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")
    rclpy.shutdown() 

if __name__ == '__main__':
    main()