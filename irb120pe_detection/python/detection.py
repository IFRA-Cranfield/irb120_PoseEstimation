#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# detection.py
# This script executes the object detection and image processing elements.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
# OpenCV:
import cv2
from cv2 import aruco
import numpy as np
# YOLOv8:
from ultralytics import YOLO

# =============================================================================== #
# CLASS -> CubeDetection:

class CubeDetection():

    def __init__(self):

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

        # Values of the CALIBRATION in x and y (mm):
        self.w = 700
        self.h = 400
