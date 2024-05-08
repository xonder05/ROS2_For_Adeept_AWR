import rclpy
from rclpy.node import Node

from interfaces.msg import LineTracking

import RPi.GPIO as GPIO

class LineTrackerNode(Node):

    def __init__(self):
        super().__init__("line_tracking_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_pin', rclpy.Parameter.Type.INTEGER),
                ('middle_pin', rclpy.Parameter.Type.INTEGER),
                ('right_pin', rclpy.Parameter.Type.INTEGER),
                ('scanning_period', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.line_pin_left = self.get_parameter('left_pin').get_parameter_value().integer_value
        self.line_pin_middle = self.get_parameter('middle_pin').get_parameter_value().integer_value
        self.line_pin_right = self.get_parameter('right_pin').get_parameter_value().integer_value
        self.scanning_period = self.get_parameter('scanning_period').get_parameter_value().integer_value
        
        self.publisher = self.create_publisher(LineTracking, "/line_visibility", 10)
        self.timer = self.create_timer(self.scanning_period, self.scanLine)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.line_pin_right,GPIO.IN)
        GPIO.setup(self.line_pin_middle,GPIO.IN)
        GPIO.setup(self.line_pin_left,GPIO.IN)

        self.get_logger().info("InitDone")

    #this literally just publishes current state of the gpio lines
    def scanLine(self):
        msg = LineTracking()
        
        if GPIO.input(self.line_pin_left) == GPIO.HIGH:
            msg.left = True
        else:
            msg.left = False

        if GPIO.input(self.line_pin_middle) == GPIO.HIGH:
            msg.middle = True
        else:
            msg.middle = False
        
        if GPIO.input(self.line_pin_right) == GPIO.HIGH:
            msg.right = True
        else:
            msg.right = False

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = LineTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()