import rclpy
from rclpy.node import Node

from interfaces.msg import SoundBlock
from std_msgs.msg import Bool

import sounddevice as sd

class SoundTransmitterNode(Node):
    def __init__(self):
        super().__init__("sound_transmitter_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('sample_rate', rclpy.Parameter.Type.INTEGER),
                ('channels', rclpy.Parameter.Type.INTEGER),
                ('blocksize', rclpy.Parameter.Type.INTEGER),
                ('audio_stream_name', rclpy.Parameter.Type.STRING),
                ('toggle_streaming_name', rclpy.Parameter.Type.STRING),
                ('start_right_away', rclpy.Parameter.Type.BOOL),
            ]
        )
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.blocksize = self.get_parameter('blocksize').get_parameter_value().integer_value
        self.audio_stream_name = self.get_parameter('audio_stream_name').get_parameter_value().string_value
        self.start_right_away = self.get_parameter('start_right_away').get_parameter_value().bool_value

        self.publisher = self.create_publisher(SoundBlock, self.audio_stream_name + "_data", 10)
        self.subscriber = self.create_subscription(Bool, self.audio_stream_name + "_toggle", self.toggle_callback, 10)

        self.stream = sd.InputStream(
            callback=self.callback, 
            samplerate=self.sample_rate, 
            channels=self.channels, 
            blocksize=self.blocksize)
        
        if self.start_right_away:
            self.stream.start()

        self.get_logger().info("InitDone")

    def toggle_callback(self, msg: Bool):
        if msg.data:
            self.stream.start()
        else:
            self.stream.stop()

    def callback(self, indata, frames, time, status):
        msg = SoundBlock()
        try:
            msg.left_channel = indata[:, 0]
            msg.right_channel = indata[:, 1]
        except: pass
        self.publisher.publish(msg)

def main():
    rclpy.init()
    try:
        node = SoundTransmitterNode()
        rclpy.spin(node)
        node.stream.close()
    except sd.PortAudioError:
        rclpy.logging.get_logger("sound_transmitter_node").error("No input audio device found, exiting")
