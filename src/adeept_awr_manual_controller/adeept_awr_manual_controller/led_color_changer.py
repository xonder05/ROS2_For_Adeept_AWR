import rclpy
from rclpy.node import Node

import sys
from adeept_awr_interfaces.srv import RGB

class LedColorChanger(Node):
    def __init__(self):
        super().__init__("led_color_changer")
        self.client = self.create_client(RGB, "change_rgb_color")
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")

        self.request = RGB.Request()

    def send_request(self, r, g, b):
        self.request.r = r
        self.request.g = g
        self.request.b = b
        
        self.get_logger().info(str(self.request.r))
        self.get_logger().info(str(self.request.g))
        self.get_logger().info(str(self.request.b))


        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
    
def main():
    rclpy.init()
    node = LedColorChanger()

    node.get_logger().info(node.get_namespace())


    node.send_request(0, 0, 0)
    node.get_logger().info("sent request")

    rclpy.shutdown()

#    while(1):
#        try:
#            r = int(input("Enter R: "))
#            g = int(input("Enter G: "))
#            b = int(input("Enter B: "))

#            node.send_request(r, g, b)

#        except ValueError:
#            pass