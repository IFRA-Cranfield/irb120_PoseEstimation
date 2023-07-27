#!/usr/bin/python3

import cv2
from cv2 import aruco
import numpy as np
import rclpy
import os
import numpy as np
import time

from ultralytics import YOLO


def main():
    rclpy.init()
    camera = cv2.VideoCapture(0) # Define the camera and its port.

    while True:
        ret, inputImg = camera.read()
        cv2.imshow("Input Image", inputImg)
        key = cv2.waitKey(1)
        if key == ord('q') or inputImg is None:
            break    
    
    if not ret:
        print("Error! Failed at capturing the image")
        rclpy.shutdown()

    if inputImg is None:
        # Error!
        print("Error! Image empty")
        rclpy.shutdown()

    # Values of the calibration x and y in mm
    w = 1050
    h = 750

    # Detection of the Aruco Markers in the input image
    param = cv2.aruco.DetectorParameters()
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    detector = aruco.ArucoDetector(aruco_dict, param)
    markerCorners, markerIds, rejectedCandidates =detector.detectMarkers(inputImg)
        
    # Visualize detection of the Aruco markers
    outputImage = inputImg.copy()
    cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds)

    # Calibration process
    src = np.zeros((4, 2), dtype=np.float32)  # Points of the corners from the input image
    dst = np.array([[0.0, 0.0], [w, 0.0], [0.0, h], [w, h]], dtype=np.float32)  # Points calibrated with w and h
    
    poly_corner = np.zeros((4, 3), dtype=np.int32)
    
    # Get the first corner & ID of the markers
    for i in range(4):
        poly_corner[i][0] = int(markerIds[i][0])
        poly_corner[i][1] = int(markerCorners[i][0][0][0])
        poly_corner[i][2] = int(markerCorners[i][0][0][1])
    
    # Arrange by ID 
    for i in range(3):
        for j in range(i + 1, 4):
            if poly_corner[i][0] > poly_corner[j][0]:
                for k in range(3):
                    aux = poly_corner[i][k]
                    poly_corner[i][k] = poly_corner[j][k]
                    poly_corner[j][k] = aux
    
    # Get the source vector
    for i in range(4):
        src[i] = np.array([poly_corner[i][1], poly_corner[i][2]], dtype=np.float32)
    
    # Get the transform matrix
    perspTransMatrix = cv2.getPerspectiveTransform(src, dst)

    # Get image of the perspective
    perspectiveImg = cv2.warpPerspective(outputImage, perspTransMatrix, (int(w), int(h)))

    cv2.imshow("Perspective Image", perspectiveImg)

    modelpath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'yolov8')

    # Load a model
    model = YOLO(modelpath + '/best.pt')  # pretrained YOLOv8n model
    names = model.names
    #Get the results
    results = model(perspectiveImg)  # return a list of Results objects

    annotated_frame =results[0].plot()

    # Process results list
    boxes = results[0].boxes
    ids = []
    for box in boxes:
        box = boxes.xyxy
        print("Box cordinates: ", boxes.xyxy)
        for c in boxes.cls:
            ids = names[int(c)]
    
    print("The ids detected: ", ids)

    x1, y1, x2, y2 = boxes[0].xyxy[0]

    x = int(x1)-5
    y = int(y1)-5
    w = int(x2)+5
    h = int(y2)+5

    classif = boxes[0].cls

    ROI = perspectiveImg[y:h, x:w]

    

# Pose estimation
    ROI = cv2.GaussianBlur(ROI,(5,5),0)
    imgHSV = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)
    lowLimit = (0, 10, 10)
    upLimit = (70, 255, 255)
    mask = cv2.inRange(imgHSV, lowLimit, upLimit)

    
# find the contours in the dilated image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    poly_contour = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 800:
            poly_contour.append(contour)
    # Get the polygonal approximation of the contour

    if not poly_contour:
        print("Contour not detected")
        rclpy.shutdown()
    
    cv2.drawContours(ROI, poly_contour, -1, (0, 255, 0), 1)
    
    for cnt in poly_contour:
        rect = cv2.minAreaRect(cnt)
        (xo, yo), (wo, ho), angle = rect
        xo = xo + x
        yo = yo + y
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.polylines(ROI, [box], True, (255,0,0), 2)
    
    print(xo)
    print(yo)

    rotation = False

    if (angle < 70 and angle > 20) or (angle > -70 and angle < -20):
        rotation = True 

    print("Angle: ", angle)
    if rotation:
        print("Rotated")
    else:
        print("Horizontal")

    cv2.imshow("Edges", mask)
    cv2.imshow("Contours", ROI)
    cv2.waitKey(0)


    cv2.destroyAllWindows()



    rclpy.shutdown()



if __name__ == "__main__":
    main()