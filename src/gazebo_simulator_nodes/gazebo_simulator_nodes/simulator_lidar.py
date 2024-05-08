import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class SimulatorLidar(Node):
    def __init__(self):
        super().__init__("simulator_lidar")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic', rclpy.Parameter.Type.STRING),
                ('ros_topic', rclpy.Parameter.Type.STRING),
                ('frame_id', rclpy.Parameter.Type.STRING),
            ]
        )
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.subscription = self.create_subscription(LaserScan, self.gazebo_topic, self.callback, 10)
        self.publisher = self.create_publisher(LaserScan, self.ros_topic , 10)

        self.get_logger().info("InitDone")

    #frame id from gazebo is not the same as the one from robot state publisher, so it has to be renamed
    def callback(self, msg: LaserScan):
        msg_out = LaserScan()
        msg_out = msg
        msg_out.header.frame_id = self.frame_id
        self.publisher.publish(msg_out)

def main():
    rclpy.init()
    node = SimulatorLidar()
    rclpy.spin(node)
    rclpy.shutdown()
