import rclpy
from rclpy.node import Node

import numpy as np
import cv2

import base64

from adeept_awr_interfaces.msg import CameraFrame
from sensor_msgs.msg import Image

class SimulatorCamera(Node):
    def __init__(self):
        super().__init__("simulator_camera")
        
        self.subscription = self.create_subscription(Image, "/simulator_camera", self.callback, 10)
        self.publisher = self.create_publisher(CameraFrame, "camera_stream", 10)
        self.get_logger().info("InitDone")

    def callback(self, msg: Image):
        image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
        _, buffer = cv2.imencode('.jpg', image_bgr)            
        base64_str = base64.b64encode(buffer).decode('utf-8')
        data_uri = 'data:image/jpeg;base64,' + base64_str

        msg = CameraFrame()
        msg.base64_data = data_uri
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SimulatorCamera()
    rclpy.spin(node)
    rclpy.shutdown()
