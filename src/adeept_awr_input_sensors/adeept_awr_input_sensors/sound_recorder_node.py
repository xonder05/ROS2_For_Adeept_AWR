import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import SoundBlock

import sounddevice as sd

class SoundRecorderNode(Node):
    def __init__(self):
        super().__init__("sound_recorder_node")
        self.publisher = self.create_publisher(SoundBlock, "audio_stream", 1)

        duration = 20
        sample_rate = 44100
        channels = 2
        blocksize = 1024

        self.stream = sd.InputStream(callback=self.callback, samplerate=sample_rate, channels=channels, blocksize=blocksize)
        self.stream.start()

        self.get_logger().info("InitDone")


    def callback(self, indata, frames, time, status):
        if status:
            print(f"Playback status: {status}")
        
        msg = SoundBlock()
        try:
            msg.left_channel = indata[:, 0]
            msg.right_channel = indata[:, 1]
        except: pass
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SoundRecorderNode()
    rclpy.spin(node)
    rclpy.shutdown()
