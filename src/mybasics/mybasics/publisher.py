#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tutorial_interfaces.msg import Num

class Publisher(Node):

    def __init__(self):
        super().__init__("publisher")
        self.number = 0
        self.cmd_vel_pub_ = self.create_publisher(Num, "topic", 10)
        self.timer_ = self.create_timer(0.5, self.push_to_topic)
        self.get_logger().info("Publisher init")

    def push_to_topic(self):
        msg = Num()
        msg.num = self.number
        self.cmd_vel_pub_.publish(msg)
        self.get_logger().info(str(self.number))
        self.number += 1


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown