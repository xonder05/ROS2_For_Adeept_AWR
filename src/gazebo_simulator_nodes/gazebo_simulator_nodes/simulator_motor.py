import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SimulatorMotor(Node):

    def __init__(self):
        super().__init__("simulator_motor")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic', rclpy.Parameter.Type.STRING),
                ('output_topic', rclpy.Parameter.Type.STRING)
            ]
        )
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.subscriber = self.create_subscription(Twist, self.input_topic, self.callback, 10)
        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        
        self.get_logger().info("InitDone")

    def callback(self, msg: Twist):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init()
    node = SimulatorMotor()
    rclpy.spin(node)
    rclpy.shutdown()