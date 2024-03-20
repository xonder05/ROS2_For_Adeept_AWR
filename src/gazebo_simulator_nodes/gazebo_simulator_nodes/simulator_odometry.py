import rclpy
from rclpy.node import Node
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class SimulatorOdometry(Node):
    
    def __init__(self):
        super().__init__('simulator_odometry')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic', rclpy.Parameter.Type.STRING),
                ('frame_id', rclpy.Parameter.Type.STRING),
                ('child_frame_id', rclpy.Parameter.Type.STRING),
            ]
        )
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.subscription = self.create_subscription(Odometry, self.gazebo_topic, self.callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("InitDone")

    def callback(self, msg: Odometry):
        transform = TransformStamped()
        
        transform.header = msg.header
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = SimulatorOdometry()
    rclpy.spin(node)
    rclpy.shutdown()
