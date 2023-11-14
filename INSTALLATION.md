## ROS2 Humble: Installation steps for irb120_PoseEstimation

All packages in this repository have been developed, executed and tested in a Ubuntu 22.04 machine with ROS 2 Humble. Please find below all the required steps to set-up a ROS 2 Humble environment in Ubuntu and install the ROS 2-based Robot Simulation and Control packages.

1. __Set-up the Ubuntu+ROS2 PC for the Robot Simulation and Control__: The steps below need to be followed in order to set-up a proper ROS2 (Humble) workspace for Robot Simulation and Control, and to be able to use ros2_SimRealRobotControl:

    (1.1) Install Ubuntu 22.04: https://ubuntu.com/desktop

    (1.2) Install Git:
      ```sh
      # In the terminal shell:
      sudo apt install git
      
      # Git account configuration:
      git config --global user.name YourUsername
      git config --global user.email YourEmail
      git config --global color.ui true
      git config --global core.editor code --wait # Visual Studio Code is recommended.
      git config --global credential.helper store
      ```
    
    (1.3) Install ROS2 Humble:
      - Follow instructions in: [ROS2 Humble Tutorials - Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
      - Source the ROS2.0 Humble installation in the .bashrc file (hidden file in /home):
        ```sh
        source opt/ros/humble/setup.bash
        ```

    (1.4) Install MoveIt!2 for ROS2 Humble ([REF: MoveIt!2 Website](https://moveit.picknik.ai/humble/index.html)):
      ```sh
      # Command for BINARY INSTALL (recommended):
      sudo apt install ros-humble-moveit
      ```

      A small improvement of the move_group_interface.h file has been developed in order to execute the Robot/Gripper triggers in this repository. Both the upgraded file and the instructions of how to implement it can be found here: [move_group_interface_improved.h](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/include)

    (1.5) Create and configure the ROS2.0 Humble ~/dev_ws environment/workspace:
      - Follow instructions in: [ROS2 Humble Tutorials - Create a ROS2 Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).
      - Source the ~/dev_ws workspace in .bashrc file:
        ```sh
        source ~/dev_ws/install/local_setup.bash
        ```

    (1.6) Install ROS2 packages, which are required for ROS2-based Robot Simulation and Control:
      ```sh
      # ROS2 Control + ROS2 Controllers:
      sudo apt install ros-humble-ros2-control
      sudo apt install ros-humble-ros2-controllers
      sudo apt install ros-humble-gripper-controllers


      # Gazebo for ROS2 Humble:
      sudo apt install gazebo
      sudo apt install ros-humble-gazebo-ros2-control
      sudo apt install ros-humble-gazebo-ros-pkgs

      # xacro:
      sudo apt install ros-humble-xacro

      # Fix cycle time issues in humble-moveit (temporary fix):
      sudo apt install ros-humble-rmw-cyclonedds-cpp # (+) Add into .bashrc file -> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      ```

</br>

2. __ABB DRIVER for ROS 2__: The installation of the [abb_ros2](https://github.com/PickNikRobotics/abb_ros2) driver is required for the control of any real ABB robot using ROS 2. 

    ```sh
    mkdir -p ~/dev_ws/src/ABBDriver
    cd ~/dev_ws/src/ABBDriver
    git clone https://github.com/PickNikRobotics/abb_ros2.git -b rolling
    sudo rosdep init
    rosdep update
    vcs import < abb_ros2/abb.repos
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    cd ~/dev_ws
    colcon build
    ```

3. __Import and install IFRA_LinkAttacher__:

    ```sh
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
    cd ~/dev_ws
    colcon build
    ```

4. __Import and install IFRA_LinkPose and IFRA_ObjectPose__:

    ```sh
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/IFRA_LinkPose.git
    git clone https://github.com/IFRA-Cranfield/IFRA_ObjectPose.git
    cd ~/dev_ws
    colcon build
    ```

5. __Import and install ros2_SimRealRobotControl__:

    ```sh
    cd ~/dev_ws/src
    git clone https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl
    cd ~/dev_ws
    colcon build
    ```