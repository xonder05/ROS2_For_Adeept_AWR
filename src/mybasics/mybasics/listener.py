#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tutorial_interfaces.msg import Num

class Subscriber(Node):

    def __init__(self):
        super().__init__("sub")
        self.pose_subscriber_ = self.create_subscription(
            Num, "topic", self.callback, 10)
        self.get_logger().info("sub init")

    def callback(self, msg: Num):
        self.get_logger().info(str(msg.num))


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()