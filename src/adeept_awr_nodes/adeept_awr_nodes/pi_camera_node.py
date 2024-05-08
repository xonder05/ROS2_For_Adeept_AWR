import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2

class PiCameraNode(Node):
    def __init__(self):
        super().__init__("pi_camera_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('video_capture', rclpy.Parameter.Type.STRING),
                ('video_resolution_width', rclpy.Parameter.Type.INTEGER),
                ('video_resolution_height', rclpy.Parameter.Type.INTEGER),
                ('video_framerate', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.video_capture = self.get_parameter('video_capture').get_parameter_value().string_value
        self.video_resolution_width = self.get_parameter('video_resolution_width').get_parameter_value().integer_value
        self.video_resolution_height = self.get_parameter('video_resolution_height').get_parameter_value().integer_value
        self.video_framerate = self.get_parameter('video_framerate').get_parameter_value().double_value
        
        self.publisher = self.create_publisher(CompressedImage, "/camera_stream", 10)
        self.timer = self.create_timer(self.video_framerate, self.get_frame)
        
        self.cap = cv2.VideoCapture(self.video_capture, cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.video_resolution_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.video_resolution_height)

        self.get_logger().info("InitDone")

    #captures image, compresses it and sends to topic
    def get_frame(self):
        ret, frame = self.cap.read()

        if ret:
            ros_image = CompressedImage()
            ros_image.format = "jpeg"
            ros_image.data = cv2.imencode('.jpg', frame)[1].tobytes()
            self.publisher.publish(ros_image)
            
def main():
    rclpy.init()
    node = PiCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()
