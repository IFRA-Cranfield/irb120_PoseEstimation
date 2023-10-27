#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

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
        self.cli = self.create_client(SetIOSignal, '/rws_client/set_io_signal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('Waiting for ABB-RWS I/O Service Server to be available...')
        print('ABB-RWS I/O Service Server detected.')
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