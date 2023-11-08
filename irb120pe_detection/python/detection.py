#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# detection.py
# This script executes the object detection and image processing elements.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
# CAMERA ROS2msg:
from sensor_msgs.msg import Image
# OpenCV:
import cv2
from cv2 import aruco
import numpy as np
# ROS2 to OpenCV -> cv_bridge:
from cv_bridge import CvBridge, CvBridgeError
# YOLOv8:
from ultralytics import YOLO
# Extra:
import os
import time

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
# CLASS -> CubeDetection:

class CubeDetection():

    def __init__(self, ENV):
        
        # Initialise CAMERA:
        if (ENV == "GAZEBO"):
            self.GzCAM_SUB = ImgSUB()
            self.InitCamGz()
        else:
            self.InitCam()

        # Values of the CALIBRATION in x and y (mm):
        self.w = 750
        self.h = 400

        # YOLO MODEL:
        modelPATH = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'yolov8')
        self.YOLOmodel = YOLO(modelPATH + '/cubeDETECTION.pt') # Pre-trained YOLOv8n model.

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

    def GetPerspectiveImg(self, ShowPerspective):

        if ShowPerspective:
            print("=== DETECTION: Calibration ===")
            print("Transforming the InputIMG into PerspectiveIMG using the ARUCO Tags...")
            print("")

        # Detection of the Aruco Markers in the input image:
        param = cv2.aruco.DetectorParameters()
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        detector = aruco.ArucoDetector(aruco_dict,param)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(self.inputImg)

        # Visualize detection of the Aruco markers:
        outputImage = self.inputImg.copy()
        cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds)

        # Calibration process:
        src = np.zeros((4, 2), dtype=np.float32)  # Points of the corners from the input image.
        dst = np.array([[0.0, 0.0], [self.w, 0.0], [0.0, self.h], [self.w, self.h]], dtype=np.float32)  # Points calibrated with w and h.
        
        poly_corner = np.zeros((4, 3), dtype=np.int32)

        # Get the first corner & ID of the markers:
        for i in range(4):
            poly_corner[i][0] = int(markerIds[i][0])
            poly_corner[i][1] = int(markerCorners[i][0][0][0])
            poly_corner[i][2] = int(markerCorners[i][0][0][1])

        # Arrange by ID: 
        for i in range(3):
            for j in range(i + 1, 4):
                if poly_corner[i][0] > poly_corner[j][0]:
                    for k in range(3):
                        aux = poly_corner[i][k]
                        poly_corner[i][k] = poly_corner[j][k]
                        poly_corner[j][k] = aux

        # Get the source vector:
        for i in range(4):
            src[i] = np.array([poly_corner[i][1], poly_corner[i][2]], dtype=np.float32)

        # Get the transform matrix:
        perspTransMatrix = cv2.getPerspectiveTransform(src, dst)

        # Get image of the perspective:
        self.perspectiveImg = cv2.warpPerspective(outputImage, perspTransMatrix, (int(self.w), int(self.h)))
        
        if ShowPerspective:

            while True:

                if self.perspectiveImg is not None:
                    self.perspectiveImgSHOW = cv2.resize(self.perspectiveImg, (750, 400))
                    cv2.imshow("IRB-120 PoseEstimation: Perspective Image", self.perspectiveImgSHOW)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    cv2.destroyWindow("IRB-120 PoseEstimation: Perspective Image")
                    break 
    
    def TestDetection(self):

        print("=== DETECTION: YOLOv8 MODEL ===")
        print("Showing the execution of the YOLOv8 model detection...")
        print("")

        while True:

            # 1. SPIN /Image topic subscriber!
            rclpy.spin_once(self.GzCAM_SUB)

            # 2. ASSIGN + Show IMG:
            self.inputImg = Gz_CAM

            # 3. Get -> perspectiveImg:
            self.GetPerspectiveImg(ShowPerspective=False)

            if self.perspectiveImg is not None:

                results = self.YOLOmodel(self.perspectiveImg)
                annotated_frame = results[0].plot()
                cv2.imshow("IRB-120 PoseEstimation: YOLO Output", annotated_frame)

            key = cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyWindow("IRB-120 PoseEstimation: YOLO Output")
                break 

    def CubeLocation(self):

        print("=== DETECTION: YOLOv8 MODEL ===")
        print("DETECTING the cube with YOLOv8 and getting its POSE...")
        print("")

        RESULT = {"x": 0.0, "y": 0.0, "yaw": 0.0, "detection": "", "success": False}

        # Run YOLOv8 model once, to load it properly:
        results = self.YOLOmodel(self.perspectiveImg)
        names = self.YOLOmodel.names

        # Run YOLOv8 detection for 3 seconds, and get RESULT:
        t_end = time.time() + 1.0
        Detected = False
        while time.time() < t_end:

            results = self.YOLOmodel(self.perspectiveImg)
            boxes = results[0].boxes

            ids = []
            for box in boxes:
                box = boxes.xyxy
                for c in boxes.cls:
                    ids.append(names[int(c)])

            if (len(ids) != 0):
                print("")
                Detected = True
                break

        if not Detected:

            print("")
            print("ERROR: The YOLOv8 model could not detect any cube in the workspace.")
            print("")
            return(RESULT)

        # AFTER DETECTION -> Get ROI (Region-of-Interest):
        x1, y1, x2, y2 = boxes[0].xyxy[0]
        x = int(x1)-3
        y = int(y1)-3
        w = int(x2)+3
        h = int(y2)+3
        ROI = self.perspectiveImg[y:h, x:w]

        # Apply mask to IMG:
        ROI = cv2.GaussianBlur(ROI,(3,3),0)
        imgHSV = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)
        lowLimit = (0, 10, 10)
        upLimit = (70, 255, 255)
        mask = cv2.inRange(imgHSV, lowLimit, upLimit)

        # Find CONTOURS:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poly_contour = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                poly_contour.append(contour)

        if not poly_contour:
            print("ERROR: No contours detected.")
            return(RESULT)

        for cnt in poly_contour:
            rect = cv2.minAreaRect(cnt)
            (xo, yo), (wo, ho), angle = rect
            xo = xo + x
            yo = yo + y

        RESULT["yaw"] = -(angle * 3.1416/180.0)
        RESULT["x"] = yo/1000 + 0.35
        RESULT["y"] = xo/1000 + 0.15
        RESULT["detection"] = "cube"
        RESULT["success"] = True

        for id in ids:

            if id == "sticker":
                RESULT["detection"] = "sticker"
                break

        return(RESULT)