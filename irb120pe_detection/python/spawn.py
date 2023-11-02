#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# spawn.py
# This script spawns the CUBES into the Gazebo Simulation environment.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
# Import /SpawnEntity Gazebo ROS2 Service:
from gazebo_msgs.srv import SpawnEntity
# Import ROS2 Messages:
from geometry_msgs.msg import Pose
# Required to load urdf file:
import os
from ament_index_python.packages import get_package_share_directory
import xacro

# =============================================================================== #
# CLASS -> SpawnEntityGz (Gazebo):

class SpawnEntityGz(Node):

    def __init__(self):

        super().__init__('irb120pe_SpawnEntity_Client')
        self.cli_SPAWN = self.create_client(SpawnEntity, "/spawn_entity")
        self.req_SPAWN = SpawnEntity.Request()

    def SPAWN(self, CUBE, POSE):

        # Load URDF:
        urdf_file = CUBE + ".urdf"
        urdf_file_path = os.path.join(get_package_share_directory('irb120pe_gazebo'), 'urdf', 'cube', urdf_file)
        xacro_file = xacro.process_file(urdf_file_path, mappings={"name": CUBE})

        # Arguments:
        self.req_SPAWN.name = CUBE
        self.req_SPAWN.xml = xacro_file.toxml()

        # POSE:
        self.req_SPAWN.initial_pose.position.x = POSE.position.x
        self.req_SPAWN.initial_pose.position.y = POSE.position.y
        self.req_SPAWN.initial_pose.position.z = POSE.position.z
        self.req_SPAWN.initial_pose.orientation.x = POSE.orientation.x
        self.req_SPAWN.initial_pose.orientation.y = POSE.orientation.y
        self.req_SPAWN.initial_pose.orientation.z = POSE.orientation.z
        self.req_SPAWN.initial_pose.orientation.w = POSE.orientation.w

        # Assign result value:
        self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

# =============================================================================== #
# CLASS -> SpawnCube:

class SpawnCube():

    def __init__(self):

        self.SpawnEntity = SpawnEntityGz()

    def SPAWN(self, CUBE, ORIENTATION):

        # Define POSE according to ORIENTATION:
        POSE = Pose()
        POSE.position.x = 0.50
        POSE.position.y = 0.53
        POSE.position.z = 0.88
        POSE.orientation.x = 0.0
        POSE.orientation.y = 0.0
        POSE.orientation.z = 0.0
        POSE.orientation.w = 0.0

        self.SpawnEntity.SPAWN(CUBE, POSE)
        while rclpy.ok():
            rclpy.spin_once(self.SpawnEntity)
            if self.SpawnEntity.future_SPAWN.done():
                try:
                    spawnRES = self.SpawnEntity.future_SPAWN.result()
                except Exception as exc:
                    print("(SpawnCube): /SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                    print("")
                    return(False)
                else:
                    if (spawnRES.success):
                        print("(SpawnCube): Successful. RESULT -> " + str(spawnRES.status_message))
                        print("")
                        return(True)
                    else:
                        print("(SpawnCube): Failed. RESULT -> " + str(spawnRES.status_message))
                        print("")
                        return(False)
