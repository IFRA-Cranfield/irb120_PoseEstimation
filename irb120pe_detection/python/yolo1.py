#!/usr/bin/python3


import cv2
import os
import rclpy

def main():

    rclpy.init()
    savepath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'src')

    camera = cv2.VideoCapture(0) #Define the camera and its port

    ret, image = camera.read()

    if not ret:
        print("Error! Failed at capturing the image")

    cv2.imwrite(os.path.join(savepath , 'frame.jpg'), image)
    camera.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()