import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

class SimulatorCamera(Node):
    def __init__(self):
        super().__init__("simulator_camera")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic', rclpy.Parameter.Type.STRING),
                ('ros_topic', rclpy.Parameter.Type.STRING)
            ]
        )
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(Image, self.gazebo_topic, self.callback, 10)
        self.publisher = self.create_publisher(CompressedImage, self.ros_topic, 10)
        
        self.get_logger().info("InitDone")

    def callback(self, msg: Image):
        
        #parse image data into 3D numpy array, encode it as .jpg and send it via compressed image
        np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        compressed_image_msg = CompressedImage()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = cv2.imencode('.jpg', np_array)[1].tobytes()
        self.publisher.publish(compressed_image_msg)

def main():
    rclpy.init()
    node = SimulatorCamera()
    rclpy.spin(node)
    rclpy.shutdown()
