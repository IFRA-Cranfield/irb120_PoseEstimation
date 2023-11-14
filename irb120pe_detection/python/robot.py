#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Irene Bernardino Sanchez - i.bernardinosanchez.854@cranfield.ac.uk          #
#           Mikel Bueno Viso         - Mikel.Bueno-Viso@cranfield.ac.uk                 #
#           Seemal Asif              - s.asif@cranfield.ac.uk                           #
#           Phil Webb                - p.f.webb@cranfield.ac.uk                         #
#                                                                                       #
#  Date: November, 2023.                                                                #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023). Object Detection and Pose Estimation within a Robot Cell. URL: https://github.com/IFRA-Cranfield/irb120_PoseEstimation

# robot.py
# This CLASSES execute Robot Movements, by calling the following ROS2 Actions:
#   - /Robmove allows the user to move the robot to a specific End-Effector pose. 
#   - /Move allows the user to execute a specific robot movement: Cartesian-Space, Joint-Space, Single Joint, Rotation... 

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# Import /Move and /RobMove ROS2 Actions:
from ros2srrc_data.action import Move
from ros2srrc_data.action import Robmove
# Import ROS2 messages:
from std_msgs.msg import String
from ros2srrc_data.msg import Action
from geometry_msgs.msg import Pose

# RobotRES variable:
from dataclasses import dataclass
@dataclass
class RobotRES:
    MESSAGE: String
    SUCCESS: bool

# Global Variable -> RES:
RES = RobotRES("null", False)

# Import WAYPOINTS class:
from waypoints import waypoints

# =============================================================================== #
# /RobMove ACTION CLIENT:

class RobMoveCLIENT(Node):

    def __init__(self):

        super().__init__('irb120pe_RobMove_Client')
        self._action_client = ActionClient(self, Robmove, 'Robmove')

        print("(/RobMove): Initialising ROS2 Action Client!")
        print("(/RobMove): Waiting for /Robmove ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("(/RobMove): /Robmove ACTION SERVER detected.")

    def send_goal(self, TYPE, SPEED, TARGET_POSE):
        
        goal_msg = Robmove.Goal()
        goal_msg.type = TYPE
        goal_msg.speed = SPEED
        goal_msg.x = TARGET_POSE.position.x
        goal_msg.y = TARGET_POSE.position.y
        goal_msg.z = TARGET_POSE.position.z
        goal_msg.qx = TARGET_POSE.orientation.x
        goal_msg.qy = TARGET_POSE.orientation.y
        goal_msg.qz = TARGET_POSE.orientation.z
        goal_msg.qw = TARGET_POSE.orientation.w
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('(/RobMove): RobMove ACTION CALL -> GOAL has been REJECTED.')
            return
        
        # print('(/RobMove): RobMove ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        RESULT = future.result().result
        RES.MESSAGE = RESULT.message
        RES.SUCCESS = RESULT.success   

# =============================================================================== #
# /Move ACTION CLIENT:

class MoveCLIENT(Node):

    def __init__(self):

        super().__init__('irb120pe_Move_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        print("(/Move): Initialising ROS2 Action Client!")
        print("(/Move): Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("(/Move): /Move ACTION SERVER detected.")

    def send_goal(self, ACTION):

        goal_msg = Move.Goal()
        goal_msg.action = ACTION.action
        goal_msg.speed = ACTION.speed
        goal_msg.movej = ACTION.movej
        goal_msg.mover = ACTION.mover
        goal_msg.movel = ACTION.movel
        goal_msg.movexyzw = ACTION.movexyzw
        goal_msg.movexyz = ACTION.movexyz
        goal_msg.moverot = ACTION.moverot
        goal_msg.moveypr = ACTION.moveypr
        goal_msg.moverp = ACTION.moverp
        goal_msg.moveg = ACTION.moveg
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('(/Move): Move ACTION CALL -> GOAL has been REJECTED.')
            return
        
        # print('(/Move): Move ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        RESULT = future.result().result
        RES.MESSAGE = RESULT.result

        if "FAILED" in RES.MESSAGE:
            RES.SUCCESS = False
        else:
            RES.SUCCESS = True

# =============================================================================== #
# ROBOT class, to execute any robot movement:

class RBT():

    def __init__(self):

        # Initialise /Move and /RobMove Action Clients:
        self.MoveClient = MoveCLIENT()
        self.RobMoveClient = RobMoveCLIENT()

        # Initialise WAYPOINTS class:
        self.Waypoints = waypoints()

    def Move_EXECUTE(self, PoseName):

        WP = Action()
        WP = self.Waypoints.RobotPose(PoseName)

        self.MoveClient.send_goal(WP)

        while rclpy.ok():

            rclpy.spin_once(self.MoveClient)

            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == True):
            
            print("Result -> " + RES.MESSAGE)
            print("")
            
            RES.MESSAGE = "null"
            RES.SUCCESS = False

        elif (RES.SUCCESS == False):
    
            print("Result -> " + RES.MESSAGE)

            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

    def RobMove_EXECUTE(self, PoseName, Type, Speed):

        WP = Pose()
        WP = self.Waypoints.RobotPose(PoseName)

        self.RobMoveClient.send_goal(Type, Speed, WP)

        while rclpy.ok():

            rclpy.spin_once(self.RobMoveClient)

            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == True):
            
            print("Result -> " + RES.MESSAGE)
            print("")
            
            RES.MESSAGE = "null"
            RES.SUCCESS = False

        elif (RES.SUCCESS == False):
    
            print("Result -> " + RES.MESSAGE)

            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()  

    def RobMove_EXECUTE_cstm(self, Type, Speed, Pose):

        self.RobMoveClient.send_goal(Type, Speed, Pose)

        while rclpy.ok():

            rclpy.spin_once(self.RobMoveClient)

            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == True):
            
            print("Result -> " + RES.MESSAGE)
            print("")
            
            RES.MESSAGE = "null"
            RES.SUCCESS = False

        elif (RES.SUCCESS == False):
    
            print("Result -> " + RES.MESSAGE)

            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()  
        

