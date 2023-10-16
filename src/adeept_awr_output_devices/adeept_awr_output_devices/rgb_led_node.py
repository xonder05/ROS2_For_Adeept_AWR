import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.srv import RGB

import time
from rpi_ws281x import *

# LED strip configuration:
LED_COUNT      = 16      # Number of LED pixels.
LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53


class RGBLedNode(Node):
    def __init__(self):
        super().__init__("rgb_led_node")
        self.srv = self.create_service(RGB, "change_rgb_color", self.changeColorCallback)

        self.LED_COUNT      = 16      # Number of LED pixels.
        self.LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
        self.LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
        self.LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        self.LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        self.LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
        self.strip.begin()
    
    def changeColorCallback(self, request, response):
        self.get_logger().info("callback")

        color = Color(request.r, request.g, request.b)
        
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
        

        return response

    def setDefaultColor(self, r, g, b):
        color = Color(r, g, b)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
    
def main():
    rclpy.init()
    node = RGBLedNode()
    node.get_logger().info(node.get_namespace())
    rclpy.spin(node)
    rclpy.shutdown()
