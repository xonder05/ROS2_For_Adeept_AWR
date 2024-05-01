import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from interfaces.srv import RGB

import subprocess

class RGBLedNode(Node):
    def __init__(self):
        super().__init__("rgb_led_node")

        self.service = self.create_service(RGB, "/change_rgb_color", self.changeColorCallback)

        self.script_path = get_package_share_directory('adeept_awr_nodes').replace("/install/adeept_awr_nodes/share/adeept_awr_nodes", "/src/adeept_awr_nodes/adeept_awr_nodes/led_controller.py")

        self.get_logger().info("InitDone")

    #calls separate script because the used library requires sudo permissions
    def changeColorCallback(self, request, response):
        subprocess.run(['sudo', self.script_path] + [str(request.r), str(request.g), str(request.b)])
        return response

def main():
    rclpy.init()
    node = RGBLedNode()
    rclpy.spin(node)
    rclpy.shutdown()
