import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.srv import RGB

import time
from rpi_ws281x import *

class RGBLedNode(Node):
    def __init__(self):
        super().__init__("rgb_led_node")
        self.srv = self.create_service(RGB, "change_rgb_color", self.changeColorCallback)

        self.LED_COUNT = 16
        self.LED_PIN = 12
        self.LED_FREQ_HZ = 800000
        self.LED_DMA = 10
        self.LED_BRIGHTNESS = 255
        self.LED_INVERT = False
        self.LED_CHANNEL = 0
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
        self.strip.begin()
    
    def changeColorCallback(self, request, response):
        color = Color(request.r, request.g, request.b)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()

    
def main():
    rclpy.init()
    node = RGBLedNode()
    rclpy.spin(node)
    rclpy.shutdown()