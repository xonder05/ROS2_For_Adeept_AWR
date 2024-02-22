import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import UltrasonicDistance
from std_msgs.msg import Bool
from std_msgs.msg import Int8

import RPi.GPIO as GPIO
import time

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")

        self.declare_parameter('tr_pin', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('ec_pin', rclpy.Parameter.Type.INTEGER)
        self.tr_pin = self.get_parameter('tr_pin').get_parameter_value().integer_value
        self.ec_pin = self.get_parameter('ec_pin').get_parameter_value().integer_value
        
        self.distance_publisher = self.create_publisher(UltrasonicDistance, "ultrasonic_distance", 10)
        self.obstacle_warning_publisher = self.create_publisher(Bool, "/obstacle_detected", 10)
        self.side_obstacle_publisher = self.create_publisher(Int8, "/side_obstacle", 10)
        self.timer = self.create_timer(0.1, self.checkdist)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tr_pin, GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.ec_pin, GPIO.IN)
        
        self.previous_distance = 0

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
        
        distance = round((t2-t1)*340/2,2)

        msg = UltrasonicDistance()
        msg.distance = distance
        self.distance_publisher.publish(msg)

        if distance - self.previous_distance > 0.2 and self.previous_distance > 0.2 and self.previous_distance < 0.7:
            msg = Int8()
            msg.data = 1
            self.side_obstacle_publisher.publish(msg)
            self.get_logger().info("right side obstacle")
        else:
            msg = Int8()
            msg.data = 0
            self.side_obstacle_publisher.publish(msg)

        self.previous_distance = distance

        if distance < 0.15:
            msg = Bool()
            msg.data = True
            self.obstacle_warning_publisher.publish(msg)
        else:
            msg = Bool()
            msg.data = False
            self.obstacle_warning_publisher.publish(msg)

def main():
    rclpy.init()
    node = UltrasonicNode()
    rclpy.spin(node)
    rclpy.shutdown()
