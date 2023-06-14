# irb120_PoseEstimation

## Installation
1. Follow installation & set-up steps in IFRA-Cranfield/ros2_SimRealRobotControl.
2. Download and install IFRA-Cranfield/irb120_PoseEstimation:
    ```sh
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/irb120_PoseEstimation
    cd ~/dev_ws
    colcon build 
    ```

## Usage
- Launch MoveIt!2 environment:
    ```sh
    ros2 launch irb120pe_moveit2 irb120pe_moveit2.launch.py
    ```
- See camera input in RVIZ: Add -> ByTopic -> CameraImage.

## IRP Author + Supervisors
- Autor: Irene Bernardino Sanchez
- Supervisor: Seemal Asif
- Second supervisor: Mikel Bueno Viso