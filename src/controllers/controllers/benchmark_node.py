import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import time
import math

class BenchmarkNode(Node):

    def __init__(self):
        super().__init__("benchmark_node")
        self.motor_publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.run()

    def run(self):
        while True:
            self.get_logger().info("Select benchmark:")
            self.get_logger().info("1 - Motor test")
            self.get_logger().info("0 - Exit")
            selection = input()

            if selection == "0":
                rclpy.shutdown()
                return

            elif selection == "1":
                self.motor_test()

    def motor_test(self):
        self.get_logger().info("Statring motor test:")

        self.get_logger().info("Drive forward for half a second -- press enter to start")
        input()
        msg = Twist()
        msg.linear.x = 0.25
        self.motor_publisher.publish(msg)
        time.sleep(0.5)
        msg = Twist()
        self.motor_publisher.publish(msg)

        self.get_logger().info("Drive backwards for half a second -- press enter to start")
        input()
        msg = Twist()
        msg.linear.x = -0.25
        self.motor_publisher.publish(msg)
        time.sleep(0.5)
        msg = Twist()
        self.motor_publisher.publish(msg)

        self.get_logger().info("Turn 90 degrees to the left -- press enter to start")
        input()
        msg = Twist()
        msg.angular.z = math.pi
        self.motor_publisher.publish(msg)
        time.sleep(0.5)
        msg = Twist()
        self.motor_publisher.publish(msg)

        self.get_logger().info("Turn 180 degrees to the right -- press enter to start")
        input()
        msg = Twist()
        msg.angular.z = -math.pi
        self.motor_publisher.publish(msg)
        time.sleep(1)
        msg = Twist()
        self.motor_publisher.publish(msg)

        self.get_logger().info("Turn 360 degrees to the left -- press enter to start")
        input()
        msg = Twist()
        msg.angular.z = math.pi
        self.motor_publisher.publish(msg)
        time.sleep(2)
        msg = Twist()
        self.motor_publisher.publish(msg)

        self.get_logger().info("--- Done ---\n")

def main():
    rclpy.init()
    node = BenchmarkNode()
    try:
        rclpy.spin(node)
    except:
        rclpy.shutdown()

    