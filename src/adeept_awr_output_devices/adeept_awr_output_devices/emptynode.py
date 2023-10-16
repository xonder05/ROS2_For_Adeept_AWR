import rclpy
from rclpy.node import Node

from adeept_awr_output_devices.testimport import printFunguje

class emptyNode(Node):
    def __init__(self):
        super().__init__("empty")

    
def main():
    rclpy.init()
    node = emptyNode()
    printFunguje(node)
    rclpy.spin(node)
    rclpy.shutdown()
