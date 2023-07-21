#!/usr/bin/python3

from ultralytics import YOLO
import cv2
import os
import rclpy

def main():

    rclpy.init()

    modelpath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'yolov8')
    filepath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'python')
    savepath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'src')

    # Load a model
    model = YOLO(modelpath + '/best.pt')  # pretrained YOLOv8n model

    path = filepath + "/perspective.jpg"
    

    # Run batched inference on a list of images
    results = model(path)  # return a list of Results objects
   
    image = cv2.imread(path)

    annotated_frame =results[0].plot()

    # Process results list
    boxes = results[0].boxes
    for box in boxes:
        box = boxes.xyxy
        ids = boxes.cls
        print("Box cordinates ", boxes.xyxy)
        print("\nBox id ", boxes.cls)

    x1, y1, x2, y2 = boxes[0].xyxy[0]

    x = int(x1)-5
    y = int(y1)-5
    w = int(x2)+5
    h = int(y2)+5

    classif = boxes[0].cls

    ROI = image[y:h, x:w]
    cv2.imwrite(os.path.join(savepath , 'ROI.jpg'), ROI)
    rclpy.shutdown()

if __name__ == '__main__':
    main()