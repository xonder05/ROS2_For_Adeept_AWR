import rclpy
from rclpy.node import Node

import sounddevice as sd
import soundfile as sf

from interfaces.msg import SoundBlock
import numpy as np

class SoundPlayerNode(Node):
    def __init__(self):
        super().__init__("sound_player_node")
        self.subscriber = self.create_subscription(SoundBlock, "audio_stream", self.data_recieved_callback, 1)
        
        sample_rate = 44100
        channels = 2
        blocksize = 1024

        self.buffer = np.empty((0, channels))
        self.stream = sd.OutputStream(callback=self.callback, samplerate=sample_rate, channels=channels, blocksize=blocksize)
        self.stream.start()

        self.get_logger().info("InitDone")

    def data_recieved_callback(self, msg: SoundBlock):
        new_data = np.vstack((msg.left_channel, msg.right_channel)).T
        self.buffer = np.concatenate((self.buffer, new_data))
        
    def callback(self, outdata, frames, time, status):
        if status:
            print(f"Playback status: {status}")

        if len(self.buffer) >= len(outdata):
            outdata[:] = self.buffer[:len(outdata)]
            self.buffer = self.buffer[len(outdata):]
        else:
            outdata[:len(self.buffer)] = self.buffer
            self.buffer = np.zeros((2048, 2))

def main():
    rclpy.init()
    node = SoundPlayerNode()
    rclpy.spin(node)
    node.stream.close()
    rclpy.shutdown()
