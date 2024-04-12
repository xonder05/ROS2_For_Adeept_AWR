import rclpy
from rclpy.node import Node

from interfaces.msg import SoundBlock
from std_msgs.msg import Bool

import numpy as np
import sounddevice as sd

class SoundReceiverNode(Node):
    def __init__(self):
        super().__init__("sound_receiver_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sample_rate', rclpy.Parameter.Type.INTEGER),
                ('channels', rclpy.Parameter.Type.INTEGER),
                ('blocksize', rclpy.Parameter.Type.INTEGER),
                ('audio_stream_name', rclpy.Parameter.Type.STRING),
                ('start_right_away', rclpy.Parameter.Type.BOOL),
            ]
        )
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.blocksize = self.get_parameter('blocksize').get_parameter_value().integer_value
        self.audio_stream_name = self.get_parameter('audio_stream_name').get_parameter_value().string_value
        self.start_right_away = self.get_parameter('start_right_away').get_parameter_value().bool_value
        
        self.subscriber = self.create_subscription(SoundBlock, self.audio_stream_name + "_data", self.data_recieved_callback, 10)
        self.subscriber = self.create_subscription(Bool, self.audio_stream_name + "_toggle", self.toggle_callback, 10)
        
        self.stream = sd.OutputStream(
            callback=self.callback, 
            samplerate=self.sample_rate, 
            channels=self.channels, 
            blocksize=self.blocksize)
        
        if self.start_right_away:
            self.stream.start()

        self.buffer = np.empty((0, self.channels))

        self.get_logger().info("InitDone")

    def toggle_callback(self, msg: Bool):
        if msg.data:
            self.stream.start()
        else:
            self.stream.stop()

    #recieved data are buffered
    def data_recieved_callback(self, msg: SoundBlock):
        new_data = np.vstack((msg.left_channel, msg.right_channel)).T
        self.buffer = np.concatenate((self.buffer, new_data))
        
    def callback(self, outdata, frames, time, status):
        if len(self.buffer) >= len(outdata):
            outdata[:] = self.buffer[:len(outdata)]
            self.buffer = self.buffer[len(outdata):]
        else:
            outdata[:len(self.buffer)] = self.buffer
            self.buffer = np.zeros((2048, 2))

def main():
    rclpy.init()
    try:
        node = SoundReceiverNode()
        rclpy.spin(node)
        node.stream.close()
    except sd.PortAudioError:
        rclpy.logging.get_logger("sound_receiver_node").error("No output audio device found, exiting")
