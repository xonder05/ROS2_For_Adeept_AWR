import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import UltrasonicDistance

import RPi.GPIO as GPIO
import time


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")
        self.publisher = self.create_publisher(UltrasonicDistance, "ultrasonic_distance", 10)
        self.timer = self.create_timer(1, self.checkdist)

        self.Tr = 11 # Pin number of input terminal of ultrasonic module
        self.Ec = 8 # Pin number of output terminal of ultrasonic module
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Tr, GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.Ec, GPIO.IN)


    def checkdist(self):
        GPIO.output(self.Tr, GPIO.HIGH) # Set the input end of the module to high level and emit an initial sound wave
        time.sleep(0.000015)
        GPIO.output(self.Tr, GPIO.LOW)
        
        while not GPIO.input(self.Ec): # When the module no longer receives the initial sound wave
            pass
        t1 = time.time() # Note the time when the initial sound wave is emitted
        
        while GPIO.input(self.Ec): # When the module receives the return sound wave
            pass
        t2 = time.time() # Note the time when the return sound wave is captured
        
        msg = UltrasonicDistance()
        msg.distance = round((t2-t1)*340/2,2)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = UltrasonicNode()
    rclpy.spin(node)
    rclpy.shutdown()
