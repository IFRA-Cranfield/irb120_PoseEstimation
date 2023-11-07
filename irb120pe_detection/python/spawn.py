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
# Import /DeleteEntity Gazebo ROS2 Service:
from gazebo_msgs.srv import DeleteEntity
# Import ROS2 Messages:
from geometry_msgs.msg import Pose
# Required to load urdf file:
import os
from ament_index_python.packages import get_package_share_directory
import xacro
# Required to spawn cube at a random pose:
import random
# Required for calculations:
import math

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
        
        # Position:
        POSE.position.x = random.uniform(0.47,0.63)
        POSE.position.y = random.uniform(0.27,0.78)
        POSE.position.z = 0.88

        # Orientation -> Will depend on the case:
        if (ORIENTATION == "FRONT"):
            roll = 0.0
            pitch = 1.5708
            yaw = random.uniform(-0.7854,0.7854)
        elif (ORIENTATION == "BOTTOM"):
            roll = 0.0
            pitch = 3.1416
            yaw = random.uniform(-0.7854,0.7854)
        # ELSE: ORIENTATION = "TOP":
        else:
            roll = 0.0
            pitch = 0.0
            yaw = random.uniform(0.0,1.5708)

        OrientationRES = EulerToQuat(roll,pitch,yaw)
        
        POSE.orientation.x = OrientationRES.orientation.x
        POSE.orientation.y = OrientationRES.orientation.y
        POSE.orientation.z = OrientationRES.orientation.z
        POSE.orientation.w = OrientationRES.orientation.w

        # Return message -> CUBE (x,y) and YAW:
        RETURN = dict()
        RETURN["x"] = POSE.position.x
        RETURN["y"] = POSE.position.y
        RETURN["yaw"] = yaw
        RETURN["success"] = False

        self.SpawnEntity.SPAWN(CUBE, POSE)
        while rclpy.ok():
            rclpy.spin_once(self.SpawnEntity)
            if self.SpawnEntity.future_SPAWN.done():
                try:
                    spawnRES = self.SpawnEntity.future_SPAWN.result()
                except Exception as exc:
                    print("(SpawnCube): /SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                    print("")
                    return(RETURN)
                else:
                    if (spawnRES.success):
                        print("(SpawnCube): Successful. RESULT -> " + str(spawnRES.status_message))
                        print("")
                        RETURN["success"] = True
                        return(RETURN)
                    else:
                        print("(SpawnCube): Failed. RESULT -> " + str(spawnRES.status_message))
                        print("")
                        return(RETURN)
                    
# =============================================================================== #
# CLASS -> DeleteEntityGz (Gazebo):

class DeleteEntityGz(Node):

    def __init__(self):

        super().__init__('irb120pe_DeleteEntity_Client')
        self.cli_DELETE = self.create_client(DeleteEntity, "/delete_entity")
        self.req_DELETE = DeleteEntity.Request()

    def DELETE(self, CUBE):

        # Arguments:
        self.req_DELETE.name = CUBE
        # Assign result value:
        self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)

# =============================================================================== #
# CLASS -> DeleteCube:

class DeleteCube():

    def __init__(self):

        self.DeleteEntity = DeleteEntityGz()

    def DELETE(self, CUBE):

        self.DeleteEntity.DELETE(CUBE)
        while rclpy.ok():
            rclpy.spin_once(self.DeleteEntity)
            if self.DeleteEntity.future_DELETE.done():
                try:
                    deleteRES = self.DeleteEntity.future_DELETE.result()
                except Exception as exc:
                    print("(DeleteCube): /DeleteEntity ROS2 Service call failed. ERROR: " + str(exc))
                    print("")
                    return(False)
                else:
                    if (deleteRES.success):
                        print("(DeleteCube): Successful. RESULT -> " + str(deleteRES.status_message))
                        print("")
                        return(True)
                    else:
                        print("(DeleteCube): Failed. RESULT -> " + str(deleteRES.status_message))
                        print("")
                        return(False)
                    
# =============================================================================== #
# FUNCTIONS -> Rotation CALCULATIONS:

def EulerToQuat(roll,pitch,yaw):

    RESULT = Pose()

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    RESULT.orientation.x = sr * cp * cy - cr * sp * sy
    RESULT.orientation.y = cr * sp * cy + sr * cp * sy
    RESULT.orientation.z = cr * cp * sy - sr * sp * cy
    RESULT.orientation.w = cr * cp * cy + sr * sp * sy

    return(RESULT)

def EEQuaternion(yaw):

    RESULT = Pose()

    # 1. Initial pose:
    Ax = 0.0
    Ay = 1.0
    Az = 0.0
    Aw = 0.0

    # 2. Get desired RELATIVE ROTATION:
    pitch = 0.0
    roll = 0.0
    cy = math.cos(-yaw * 0.5)
    sy = math.sin(-yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    Bx = sr * cp * cy - cr * sp * sy
    By = cr * sp * cy + sr * cp * sy
    Bz = cr * cp * sy - sr * sp * cy
    Bw = cr * cp * cy + sr * sp * sy

    # 3. Quaternion MULTIPLICATION:
    RESULT.orientation.x = Aw*Bx + Ax*Bw + Ay*Bz - Az*By
    RESULT.orientation.y = Aw*By - Ax*Bz + Ay*Bw + Az*Bx
    RESULT.orientation.z = Aw*Bz + Ax*By - Ay*Bx + Az*Bw
    RESULT.orientation.w = Aw*Bw - Ax*Bx - Ay*By - Az*Bz

    return(RESULT)