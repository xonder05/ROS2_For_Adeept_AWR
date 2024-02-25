import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import time

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('tr_pin', rclpy.Parameter.Type.INTEGER),
                ('ec_pin', rclpy.Parameter.Type.INTEGER),
                ('obstacle_warning_distance', rclpy.Parameter.Type.DOUBLE),
                ('side_obstacle_minimum_detection_distance', rclpy.Parameter.Type.DOUBLE),
                ('side_obstacle_maximum_detection_distance', rclpy.Parameter.Type.DOUBLE)
            ]
        )
        self.tr_pin = self.get_parameter('tr_pin').get_parameter_value().integer_value
        self.ec_pin = self.get_parameter('ec_pin').get_parameter_value().integer_value
        self.obstacle_warning_distance = self.get_parameter('obstacle_warning_distance').get_parameter_value().double_value
        self.side_obstacle_minimum_detection_distance = self.get_parameter('side_obstacle_minimum_detection_distance').get_parameter_value().double_value
        self.side_obstacle_maximum_detection_distance = self.get_parameter('side_obstacle_maximum_detection_distance').get_parameter_value().double_value

        self.distance_publisher = self.create_publisher(Float32, "/ultrasonic_distance", 10)
        self.obstacle_warning_publisher = self.create_publisher(Bool, "/ultrasonic_obstacle_warning", 10)
        self.obstacle_disappearance_warning_publisher = self.create_publisher(Bool, "/ultrasonic_obstacle_disappearance_warning", 10)
        self.timer = self.create_timer(0.1, self.checkdist)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tr_pin, GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.ec_pin, GPIO.IN)
        
        self.previous_distance = 0

        self.get_logger().info("InitDone")

    def checkdist(self):

        #trigger distance mesurement on the sensor
        GPIO.output(self.tr_pin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.tr_pin, GPIO.LOW)
        
        #listening to the sensor
        while not GPIO.input(self.ec_pin): # When the module no longer receives the initial sound wave
            pass
        t1 = time.time() # Note the time when the initial sound wave is emitted
        
        while GPIO.input(self.ec_pin): # When the module receives the return sound wave
            pass
        t2 = time.time() # Note the time when the return sound wave is captured
        
        #calculate the distance based on the speed of sound proparation in air
        distance = round((t2-t1)*340/2,2)

        #publish calculated distance
        msg = Float32()
        msg.data = distance
        self.distance_publisher.publish(msg)

        #publish warning about close obstacle
        msg = Bool()
        if distance < self.obstacle_warning_distance:
            msg.data = True
        else:
            msg.data = False
        self.obstacle_warning_publisher.publish(msg)

        #publish warning about obstacle that is no longer visible but is on trajectory that could lead to colision with the robot
        msg = Bool()
        if distance - self.previous_distance > 0.2 and self.previous_distance > self.side_obstacle_minimum_detection_distance and self.previous_distance < self.side_obstacle_maximum_detection_distance:
            msg.data = True
        else:
            msg.data = False
        self.obstacle_disappearance_warning_publisher.publish(msg)

        self.previous_distance = distance

def main():
    rclpy.init()
    node = UltrasonicNode()
    rclpy.spin(node)
    rclpy.shutdown()
