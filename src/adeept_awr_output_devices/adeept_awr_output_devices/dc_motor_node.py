import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import DriveDirections
from adeept_awr_output_devices import dc_motor

import time

class DCMotorNode(Node):

    def __init__(self):
        super().__init__("dc_motor_node")
        self.motor = dc_motor.DCMotor()

        self.subscriber = self.create_subscription(DriveDirections, "drive_directions", self.callback, 10)
        
        self.get_logger().info("InitDone")

    def callback(self, msg: DriveDirections):

        if (msg.direction == 8):
            self.forward(msg.speed)
        elif (msg.direction == 2):
            self.back(msg.speed)
        elif (msg.direction == 4):
            self.left(msg.speed)
        elif (msg.direction == 6):
            self.right(msg.speed)
        else:
            self.stop()

    def forward(self, speed):
        self.motor.left_side_motor(1, speed)
        self.motor.right_side_motor(1, speed)

    def back(self, speed):
        self.motor.left_side_motor(-1, speed)
        self.motor.right_side_motor(-1, speed)

    def left(self, speed):
        self.motor.left_side_motor(-1, speed)
        self.motor.right_side_motor(1, speed)

    def right(self, speed):
        self.motor.left_side_motor(1, speed)
        self.motor.right_side_motor(-1, speed)

    def stop(self):
        self.motor.stopMotors()

def main(args=None):
    rclpy.init(args=args)
    node = DCMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()