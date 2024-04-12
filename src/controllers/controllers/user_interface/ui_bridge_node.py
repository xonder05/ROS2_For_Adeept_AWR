from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from interfaces.msg import LineTracking
from std_srvs.srv import SetBool 
from interfaces.srv import RGB

import numpy as np

class UiBridgeNode(Node):
    def __init__(self):
        super().__init__('ui_bridge_node')

        self.subscriber = self.create_subscription(CompressedImage, "/camera_stream", self.camera_callback, 10)
        self.subscriber = self.create_subscription(Float32, "/ultrasonic_distance", self.ultrasonic_callback, 10)
        self.subscriber = self.create_subscription(LineTracking, "/line_visibility", self.line_visibility_callback, 10)
        self.subscriber = self.create_subscription(String, "/wandering_state", self.wandering_state_callback, 10)
        self.subscriber = self.create_subscription(String, "/line_following_state", self.line_following_state_callback, 10)

        self.audio_out_toggle = self.create_publisher(Bool, "/control_transmitter_robot_receiver_toggle", 10)
        self.audio_in_toggle = self.create_publisher(Bool, "/robot_transmitter_control_receiver_toggle", 10)

        self.gamepad_toggle_client = self.create_client(SetBool, "/toggle_gamepad")
        self.line_following_toggle_client = self.create_client(SetBool, "/toggle_line_following")
        self.wandering_toggle_client = self.create_client(SetBool, "/toggle_wandering")
        self.keyboard_toggle_client = self.create_client(SetBool, "/toggle_keyboard")
        self.rgb_led_client = self.create_client(RGB, "/change_rgb_color")

        self.get_logger().info("InitDone")

    def put_window(self, qt):
        self.qt = qt

    def ultrasonic_callback(self, msg: Float32):
        self.qt.update_ultrasonic_label(msg.data)

    def line_visibility_callback(self, msg: LineTracking):
        self.qt.update_line_tracking_label(msg.left, msg.middle, msg.right)

    def wandering_state_callback(self, msg: String):
        self.qt.update_wandering_state_label(msg.data)

    def line_following_state_callback(self, msg: String):
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

    def camera_callback(self, msg: CompressedImage):
        np_array = np.frombuffer(msg.data, np.uint8)
        self.qt.update_image(np_array)

    def audio_microphone_toggle(self, mic_state):
        msg = Bool()
        if mic_state == "muted":
            msg.data = True
        else:
            msg.data = False
        self.audio_out_toggle.publish(msg)

    def audio_speaker_toggle(self, speaker_state):
        msg = Bool()
        if speaker_state == "muted":
            msg.data = True
        else:
            msg.data = False
        self.audio_in_toggle.publish(msg)

    def change_rgb_color(self, color):
        while not self.rgb_led_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")

        request = RGB.Request()
        request.r = color.red()
        request.g = color.green()
        request.b = color.blue()
        self.rgb_led_client.call_async(request)