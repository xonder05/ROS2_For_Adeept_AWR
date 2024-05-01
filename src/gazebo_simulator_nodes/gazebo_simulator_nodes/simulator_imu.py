import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

import math
import transforms3d

class SimulatorImu(Node):
    def __init__(self):
        super().__init__("simulator_imu")
        
        self.subscription = self.create_subscription(Imu, "/simulator_imu", self.callback, 10)
        self.publisher = self.create_publisher(Twist, "/imu_data", 10)

        self.previous_rotation = 0
        self.cumulative_rotation = 0

        self.get_logger().info("InitDone")

    def callback(self, msg):
        #convert quaternions
        euler = transforms3d.euler.quat2euler([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

        #only interested in z axis
        current_rotation = euler[2]

        rotation_difference = current_rotation - self.previous_rotation

        #the data are from -3.14 to +3.14, so passing across this border must be handeled
        if rotation_difference > math.pi:
            rotation_difference -= 2 * math.pi
        elif rotation_difference < -math.pi:
            rotation_difference += 2 * math.pi

        #save absolute rotation required by other nodes
        self.cumulative_rotation += rotation_difference

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.cumulative_rotation
        self.publisher.publish(msg)

        self.previous_rotation = current_rotation

def main():
    rclpy.init()
    node = SimulatorImu()
    rclpy.spin(node)
    rclpy.shutdown()
