import rclpy
from rclpy.node import Node

import sys
from adeept_awr_interfaces.srv import RGB

class LedColorChanger(Node):
    def __init__(self):
        super().__init__("led_color_changer")
        self.client = self.create_client(RGB, "change_rgb_color")
        self.request = RGB.Request()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")

    def send_request(self, r, g, b):
        self.request.r = r
        self.request.g = g
        self.request.b = b
        
        self.future = self.client.call_async(self.request)
    
def main():
    rclpy.init()
    node = LedColorChanger()

    while(1):
        try:
            user_input = int(input("Enter R: "))
            user_input = int(input("Enter G: "))
            user_input = int(input("Enter B: "))

            node.send_goal(user_input)

        except ValueError:
            pass