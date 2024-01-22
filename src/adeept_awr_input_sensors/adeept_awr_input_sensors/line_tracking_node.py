import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
import time

from adeept_awr_interfaces.msg import LineTracking

class LineTrackerNode(Node):

    def __init__(self):
        super().__init__("line_tracking_node")

        self.declare_parameter('left_pin', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('middle_pin', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('right_pin', rclpy.Parameter.Type.INTEGER)
        self.line_pin_left = self.get_parameter('left_pin').get_parameter_value().integer_value
        self.line_pin_middle = self.get_parameter('middle_pin').get_parameter_value().integer_value
        self.line_pin_right = self.get_parameter('right_pin').get_parameter_value().integer_value
        
        self.timer = self.create_timer(1, self.scanLine)
        self.publisher = self.create_publisher(LineTracking, "line_visibility", 10)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.line_pin_right,GPIO.IN)
        GPIO.setup(self.line_pin_middle,GPIO.IN)
        GPIO.setup(self.line_pin_left,GPIO.IN)

        self.get_logger().info("InitDone")

    def scanLine(self):
        status_right = GPIO.input(self.line_pin_right)
        status_middle = GPIO.input(self.line_pin_middle)
        status_left = GPIO.input(self.line_pin_left)

        msg = LineTracking()
        msg.middle = False
        msg.left = False
        msg.right = False

        if status_middle == 1:
            msg.middle = True
        if status_left == 1:
            msg.left = True
        if status_right == 1:
            msg.right = True

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LineTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()