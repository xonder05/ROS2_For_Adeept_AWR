import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from adeept_awr_interfaces.msg import CameraFrame
from adeept_awr_interfaces.msg import UltrasonicDistance
from adeept_awr_interfaces.msg import LineTracking
from std_srvs.srv import SetBool 
from std_msgs.msg import String

class UiBridgeNode(Node):
    def __init__(self):
        super().__init__('ui_bridge_node')

        self.subscriber = self.create_subscription(CameraFrame, "/camera_stream", self.camera_callback, 10)
        self.first = True

        self.subscriber = self.create_subscription(UltrasonicDistance, "/ultrasonic_distance", self.ultrasonic_callback, 10)
        self.subscriber = self.create_subscription(LineTracking, "/line_visibility", self.line_visibility_callback, 10)
        
        self.subscriber = self.create_subscription(String, "/wandering_state", self.wandering_state_callback, 10)
        self.subscriber = self.create_subscription(String, "/line_tracking_state", self.line_tracking_state_callback, 10)

        self.gamepad_toggle_client = self.create_client(SetBool, "/toggle_gamepad")
        self.line_following_toggle_client = self.create_client(SetBool, "/toggle_line_following")
        self.wandering_toggle_client = self.create_client(SetBool, "/toggle_wandering")
        self.keyboard_toggle_client = self.create_client(SetBool, "/toggle_keyboard")

    def put_window(self, qt):
        self.qt = qt

    def ultrasonic_callback(self, msg: UltrasonicDistance):
        self.qt.update_ultrasonic_label(msg.distance)

    def line_visibility_callback(self, msg: LineTracking):
        self.qt.update_line_tracking_label(msg.left, msg.middle, msg.right)

    def wandering_state_callback(self, msg: String):
        self.qt.update_wandering_state_label(msg.data)

    def line_tracking_state_callback(self, msg: String):
        self.qt.update_line_tracking_state_label(msg.data)

    def gamepad_toggle(self, action):        
        while not self.gamepad_toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")

        request = SetBool.Request()
        request.data = action
        self.gamepad_toggle_client.call_async(request)

    def line_following_toggle(self, action):
        while not self.line_following_toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")

        request = SetBool.Request()
        request.data = action
        self.line_following_toggle_client.call_async(request)

    def wandering_toggle(self, action):
        while not self.wandering_toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")

        request = SetBool.Request()
        request.data = action
        self.wandering_toggle_client.call_async(request)

    def keyboard_toggle(self, action):
        while not self.keyboard_toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")

        request = SetBool.Request()
        request.data = action
        self.keyboard_toggle_client.call_async(request)

    def camera_callback(self, msg: CameraFrame):
        self.qt.update_image(msg.base64_data)