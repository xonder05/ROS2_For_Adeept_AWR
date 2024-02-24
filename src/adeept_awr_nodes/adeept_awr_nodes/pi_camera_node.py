import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2

class PiCameraNode(Node):
    def __init__(self):
        super().__init__("pi_camera_node")
        
        self.declare_parameter('video_capture', rclpy.Parameter.Type.STRING)
        self.declare_parameter('video_resolution_width', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('video_resolution_height', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('video_framerate', rclpy.Parameter.Type.DOUBLE)

        self.publisher = self.create_publisher(CompressedImage, "camera_stream", 10)
        self.timer = self.create_timer(self.get_parameter('video_framerate').get_parameter_value().double_value, 
                                       self.get_frame)
        
        self.cap = cv2.VideoCapture(self.get_parameter('video_capture').get_parameter_value().string_value, 
                                    cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,
                     self.get_parameter('video_resolution_width').get_parameter_value().integer_value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 
                     self.get_parameter('video_resolution_height').get_parameter_value().integer_value)

        self.get_logger().info("InitDone")

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
