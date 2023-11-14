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

# gripper.py
# This function opens/closes the Schunk EGP-64 gripper in the ABB IRB-120 Robot.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
# Import ROS2 Services:
from abb_robot_msgs.srv import SetIOSignal

# =============================================================================== #
# ABB Robot I/O - ROS2 Service Client:

class abbRWS_IO(Node):

    def __init__(self):

        super().__init__('abbRWS_IO_client')

        print("(abbRWS_IO): Initialising ROS2 Service Client!")
        self.cli = self.create_client(SetIOSignal, '/rws_client/set_io_signal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('(abbRWS_IO): Waiting for ABB-RWS I/O Service Server to be available...')

        print('(abbRWS_IO): ABB-RWS I/O Service Server detected.')
        self.req = SetIOSignal.Request()

    def send_request(self, signal, value):
        self.req.signal = signal
        self.req.value = value
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

    def OPEN(self):
        signal = "CloseGripper"
        value = "0"
        self.send_request(signal,value)
        signal = "OpenGripper"
        value = "1"
        self.send_request(signal,value)

    def CLOSE(self):
        signal = "OpenGripper"
        value = "0"
        self.send_request(signal,value)
        signal = "CloseGripper"
        value = "1"
        self.send_request(signal,value)