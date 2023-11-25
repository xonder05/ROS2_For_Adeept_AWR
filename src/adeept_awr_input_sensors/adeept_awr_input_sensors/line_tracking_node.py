import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
import time


# Hunt module output pin
line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20

class LineTrackerNode(Node):

    def __init__(self):
        super().__init__("line_tracking_node")

    def setup(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(line_pin_right,GPIO.IN)
        GPIO.setup(line_pin_middle,GPIO.IN)
        GPIO.setup(line_pin_left,GPIO.IN)
        
    def run(self):
        status_right = GPIO.input(line_pin_right)
        status_middle = GPIO.input(line_pin_middle)
        status_left = GPIO.input(line_pin_left)

        if status_middle == 1:
            print("middle")
        if status_left == 1:
            print("left")
        if status_right == 1:
            print("right")

        print("----------")

def main():
    rclpy.init()
    node = LineTrackerNode()
    node.setup()
    
    while 1:
        node.run()
        time.sleep(1)

    rclpy.shutdown()