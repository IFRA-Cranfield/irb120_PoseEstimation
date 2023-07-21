from ultralytics import YOLO

import cv2
import os

filepath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'yolov8')


# Load a model
model = YOLO(filepath + '/best.pt')  # pretrained YOLOv8n model

# Run batched inference on a list of images
results = model(filepath + '/perspective.jpg')  # return a list of Results objects

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bbox outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Class probabilities for classification outputs


for box in boxes:
	box = boxes.xyxy
	ids = boxes.cls
	print("Box cordinates ", boxes.xywh)
	print("\nBox id ", boxes.cls)

