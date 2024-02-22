import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.srv import RGB

import subprocess

class RGBLedNode(Node):
    def __init__(self):
        super().__init__("rgb_led_node")
        self.srv = self.create_service(RGB, "change_rgb_color", self.changeColorCallback)
        
        self.get_logger().info("InitDone")

    def changeColorCallback(self, request, response):
        self.get_logger().info("callback")

        subprocess.run(['sudo', 'python3', "/home/pi/ros2_ws/src/adeept_awr_nodes/adeept_awr_nodes/led_controller.py"] + [str(request.r), str(request.g), str(request.b)])

        return response
    
def main():
    rclpy.init()
    node = RGBLedNode()
    rclpy.spin(node)
    rclpy.shutdown()
