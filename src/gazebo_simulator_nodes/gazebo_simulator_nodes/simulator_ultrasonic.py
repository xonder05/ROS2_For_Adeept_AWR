import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from adeept_awr_interfaces.msg import UltrasonicDistance
from std_msgs.msg import Bool
from std_msgs.msg import Int8

class SimulatorUltrasonic(Node):

    def __init__(self):
        super().__init__("simulator_ultrasonic")
        
        self.subscriber = self.create_subscription(LaserScan, "/simulator_ultrasonic", self.callback, 10)
        self.distance_publisher = self.create_publisher(UltrasonicDistance, "ultrasonic_distance", 10)
        self.obstacle_warning_publisher = self.create_publisher(Bool, "/obstacle_detected", 10)
        self.side_obstacle_publisher = self.create_publisher(Int8, "/side_obstacle", 10)
        self.previous_distance = 0
        self.get_logger().info("InitDone")

    def callback(self, msg: LaserScan):
        left = msg.ranges[0]
        middle = msg.ranges[1]
        right = msg.ranges[2]
        
        self.get_logger().info(f"left: {left}")
        self.get_logger().info(f"middle {middle}")
        self.get_logger().info(f"right {right}")
        
        distance = min(left, min(middle, right))

        msg = UltrasonicDistance()
        msg.distance = distance
        self.distance_publisher.publish(msg)

        if distance < 0.15:
            msg = Bool()
            msg.data = True
            self.obstacle_warning_publisher.publish(msg)
        else:
            msg = Bool()
            msg.data = False
            self.obstacle_warning_publisher.publish(msg)

        if distance - self.previous_distance > 0.2 and self.previous_distance < 0.7 and self.previous_distance > 0.3:
            msg = Int8()
            msg.data = 1
            self.side_obstacle_publisher.publish(msg)
            self.get_logger().info("side obstacle")

        else:
            msg = Int8()
            msg.data = 0
            self.side_obstacle_publisher.publish(msg)

        self.previous_distance = distance


def main(args=None):
    rclpy.init()
    node = SimulatorUltrasonic()
    rclpy.spin(node)
    rclpy.shutdown()