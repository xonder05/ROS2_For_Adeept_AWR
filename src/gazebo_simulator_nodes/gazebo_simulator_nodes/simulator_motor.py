import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SimulatorMotor(Node):

    def __init__(self):
        super().__init__("simulator_motor")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic', rclpy.Parameter.Type.STRING),
                ('ros_topic', rclpy.Parameter.Type.STRING)
            ]
        )
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value

        self.subscriber = self.create_subscription(Twist, self.gazebo_topic, self.callback, 10)
        self.publisher = self.create_publisher(Twist, self.ros_topic, 10)
        
        self.get_logger().info("InitDone")

    #just topic name change
    def callback(self, msg: Twist):
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SimulatorMotor()
    rclpy.spin(node)
    rclpy.shutdown()