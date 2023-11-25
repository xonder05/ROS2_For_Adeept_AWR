import rclpy
from rclpy.node import Node
import cv2

import base64

from adeept_awr_interfaces.msg import CameraFrame

class PiCameraNode(Node):
    def __init__(self):
        super().__init__("pi_camera_node")
        self.publisher = self.create_publisher(CameraFrame, "camera_stream", 10)
        self.timer = self.create_timer(0.05, self.get_frame)
        
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)


    def get_frame(self):
        ret, frame = self.cap.read()

        if ret:
            _, buffer = cv2.imencode('.jpg', frame)            
            base64_str = base64.b64encode(buffer).decode('utf-8')
            data_uri = 'data:image/jpeg;base64,' + base64_str
            msg = CameraFrame()
            msg.base64_data = data_uri
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PiCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()
