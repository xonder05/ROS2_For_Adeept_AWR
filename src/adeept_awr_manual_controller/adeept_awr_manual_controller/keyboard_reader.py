#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import DriveDirections

import getch

class KeyboardReader(Node):
    def __init__(self):
        super().__init__("keyboard_reader")
        self.publisher = self.create_publisher(DriveDirections, "drive_directions", 10)

    def keyboard_listener(self):
        while True:
            try:
                char = getch.getch()
                if (char in ["w","s","a","d"]):
                    output = DriveDirections()
                    output.direction = ord(char)
                    self.publisher.publish(output)
                    
                    self.get_logger().info(char)
            except (OverflowError, ValueError):
                pass

def main():
    rclpy.init()
    node = KeyboardReader()
    node.keyboard_listener()
    #rclpy.spin(node)
    rclpy.shutdown()