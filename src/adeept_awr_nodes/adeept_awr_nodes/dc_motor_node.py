import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

class DCMotorNode(Node):

    def __init__(self):
        super().__init__("dc_motor_node")
        
        self.init_params()
        
        self.subscriber = self.create_subscription(Twist, "/cmd_vel", self.callback, 10)
        self.obstacle_subscriber = self.create_subscription(Bool, "/ultrasonic_obstacle_warning", self.obstacle_callback, 10)
        
        self.init_gpio()
        
        self.obstacle = False
        
        self.get_logger().info("InitDone")

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('motor_left_enable_pin', rclpy.Parameter.Type.INTEGER),
                ('motor_left_backward_pin', rclpy.Parameter.Type.INTEGER),
                ('motor_left_forward_pin', rclpy.Parameter.Type.INTEGER),
                ('motor_right_enable_pin', rclpy.Parameter.Type.INTEGER),
                ('motor_right_backward_pin', rclpy.Parameter.Type.INTEGER),
                ('motor_right_forward_pin', rclpy.Parameter.Type.INTEGER),
                ('half_distance_between_wheels', rclpy.Parameter.Type.DOUBLE),
                ('wheel_radius', rclpy.Parameter.Type.DOUBLE),
                ('max_motor_rotation_speed', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.motor_left_enable_pin = self.get_parameter('motor_left_enable_pin').get_parameter_value().integer_value
        self.motor_left_backward_pin = self.get_parameter('motor_left_backward_pin').get_parameter_value().integer_value
        self.motor_left_forward_pin = self.get_parameter('motor_left_forward_pin').get_parameter_value().integer_value
        self.motor_right_enable_pin = self.get_parameter('motor_right_enable_pin').get_parameter_value().integer_value
        self.motor_right_backward_pin = self.get_parameter('motor_right_backward_pin').get_parameter_value().integer_value
        self.motor_right_forward_pin = self.get_parameter('motor_right_forward_pin').get_parameter_value().integer_value
        self.half_distance_between_wheels = self.get_parameter('half_distance_between_wheels').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.max_motor_rotation_speed = self.get_parameter('max_motor_rotation_speed').get_parameter_value().double_value

    def init_gpio(self):
        GPIO.cleanup()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.motor_left_enable_pin, GPIO.OUT)
        GPIO.setup(self.motor_left_backward_pin, GPIO.OUT)
        GPIO.setup(self.motor_left_forward_pin, GPIO.OUT)
        GPIO.setup(self.motor_right_enable_pin, GPIO.OUT)
        GPIO.setup(self.motor_right_backward_pin, GPIO.OUT)
        GPIO.setup(self.motor_right_forward_pin, GPIO.OUT)
        
        self.stopMotors() #set all gpio to low

        global pwm_A, pwm_B
        try:
            pwm_A = GPIO.PWM(self.motor_left_enable_pin, 1000)
            pwm_B = GPIO.PWM(self.motor_right_enable_pin, 1000)
        except:
            pass
    
    def stopMotors(self):
        GPIO.output(self.motor_left_backward_pin, GPIO.LOW)
        GPIO.output(self.motor_left_forward_pin, GPIO.LOW)
        GPIO.output(self.motor_right_backward_pin, GPIO.LOW)
        GPIO.output(self.motor_right_forward_pin, GPIO.LOW)
        GPIO.output(self.motor_left_enable_pin, GPIO.LOW)
        GPIO.output(self.motor_right_enable_pin, GPIO.LOW)

    #controlling h-bridge circuite
    def left_side_motor(self, speed):
        if speed >= 0:
            GPIO.output(self.motor_left_backward_pin, GPIO.HIGH)
            GPIO.output(self.motor_left_forward_pin, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        
        elif speed < 0:
            GPIO.output(self.motor_left_backward_pin, GPIO.LOW)
            GPIO.output(self.motor_left_forward_pin, GPIO.HIGH)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(-speed)

    #controlling h-bridge circuite
    def right_side_motor(self, speed):
        if speed >= 0:
            GPIO.output(self.motor_right_backward_pin, GPIO.HIGH)
            GPIO.output(self.motor_right_forward_pin, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        
        elif speed < 0:
            GPIO.output(self.motor_right_backward_pin, GPIO.LOW)
            GPIO.output(self.motor_right_forward_pin, GPIO.HIGH)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(-speed)

    def callback(self, msg: Twist):

        #no need to do calculations if the request is to stop
        if msg.linear.x == 0 and msg.angular.z == 0:
            self.stopMotors()
            return

        #don't move forward when obstacle is detected
        if self.obstacle and msg.linear.x > 0:
            return

        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        #calculate speed for each side from linear and angular components
        left_motor_speed = (linear_speed - angular_speed * self.half_distance_between_wheels) / self.wheel_radius
        right_motor_speed = (linear_speed + angular_speed * self.half_distance_between_wheels) / self.wheel_radius

        #crop speeds that are more than what motors can handle
        left_motor_speed = max(min(left_motor_speed, self.max_motor_rotation_speed), -self.max_motor_rotation_speed)
        right_motor_speed = max(min(right_motor_speed, self.max_motor_rotation_speed), -self.max_motor_rotation_speed)

        #map speed to pwm duty cycle value
        normalized_left_motor_speed = (left_motor_speed / self.max_motor_rotation_speed) * 100
        normalized_right_motor_speed = (right_motor_speed / self.max_motor_rotation_speed) * 100

        self.left_side_motor(normalized_left_motor_speed)
        self.right_side_motor(normalized_right_motor_speed)

    def obstacle_callback(self, msg: Bool):
        if msg.data:
            if not self.obstacle:
                self.stopMotors()
            self.obstacle = True
        else:
            self.obstacle = False 

def main():
    rclpy.init()
    node = DCMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()