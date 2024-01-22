import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import UltrasonicDistance

import RPi.GPIO as GPIO
import time

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")

        self.declare_parameter('tr_pin', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('ec_pin', rclpy.Parameter.Type.INTEGER)
        self.tr_pin = self.get_parameter('tr_pin').get_parameter_value().integer_value
        self.ec_pin = self.get_parameter('ec_pin').get_parameter_value().integer_value
        
        self.publisher = self.create_publisher(UltrasonicDistance, "ultrasonic_distance", 10)
        self.timer = self.create_timer(1, self.checkdist)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tr_pin, GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.ec_pin, GPIO.IN)
        
        self.get_logger().info("InitDone")

    def checkdist(self):
        GPIO.output(self.tr_pin, GPIO.HIGH) # Set the input end of the module to high level and emit an initial sound wave
        time.sleep(0.000015)
        GPIO.output(self.tr_pin, GPIO.LOW)
        
        while not GPIO.input(self.ec_pin): # When the module no longer receives the initial sound wave
            pass
        t1 = time.time() # Note the time when the initial sound wave is emitted
        
        while GPIO.input(self.ec_pin): # When the module receives the return sound wave
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
