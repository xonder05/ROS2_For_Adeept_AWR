import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import RPi.GPIO as GPIO
GPIO.cleanup()

class DCMotorNode(Node):

    def __init__(self):
        super().__init__("dc_motor_node")
        self.init_params()
        self.subscriber = self.create_subscription(Twist, "drive_directions", self.callback, 10)
        self.obstacle_subscriber = self.create_subscription(Bool, "/obstacle_detected", self.obstacle_callback, 10)
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
                ('motor_right_forward_pin', rclpy.Parameter.Type.INTEGER)
            ]
        )
        
        self.motor_left_enable_pin = self.get_parameter('motor_left_enable_pin').get_parameter_value().integer_value
        self.motor_left_backward_pin = self.get_parameter('motor_left_backward_pin').get_parameter_value().integer_value
        self.motor_left_forward_pin = self.get_parameter('motor_left_forward_pin').get_parameter_value().integer_value
        self.motor_right_enable_pin = self.get_parameter('motor_right_enable_pin').get_parameter_value().integer_value
        self.motor_right_backward_pin = self.get_parameter('motor_right_backward_pin').get_parameter_value().integer_value
        self.motor_right_forward_pin = self.get_parameter('motor_right_forward_pin').get_parameter_value().integer_value

    def init_gpio(self):
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

    def left_side_motor(self, speed):
        if speed > 0:
            GPIO.output(self.motor_left_backward_pin, GPIO.HIGH)
            GPIO.output(self.motor_left_forward_pin, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        
        elif speed < 0:
            GPIO.output(self.motor_left_backward_pin, GPIO.LOW)
            GPIO.output(self.motor_left_forward_pin, GPIO.HIGH)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(-speed)

    def right_side_motor(self, speed):
        if speed > 0:
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
        if msg.linear.x == 0 and msg.angular.z == 0:
            self.stopMotors()
            return

        if self.obstacle and msg.linear.x > 0:
            return

        HALF_DISTANCE_BETWEEN_WHEELS = 0.07 #m
        WHEEL_RADIUS = 0.035 #m
        MAX_MOTOR_ROTATION_SPEED = 8.77 #rad/s
        MAX_LINEAR_SPEED = 0.30695 #m/s
        MAX_ANGULAR_SPEED = 4.385 #rad/s

        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        self.get_logger().info("-------------")
        self.get_logger().info(str(linear_speed))
        self.get_logger().info(str(angular_speed))

        linear_percentage = abs(linear_speed) / MAX_LINEAR_SPEED * 100
        angular_percentage = abs(angular_speed) / MAX_ANGULAR_SPEED * 100

        #if required speeds exceed motor capabilities
        if angular_percentage > 100:
            linear_speed = 0
            angular_speed = MAX_ANGULAR_SPEED

        elif linear_percentage + angular_percentage > 100:
            linear_speed = (linear_speed / linear_percentage) * (100 - angular_percentage)

        self.get_logger().info(str(linear_speed))
        self.get_logger().info(str(angular_speed))

        left_motor_speed = (linear_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_motor_speed = (linear_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.get_logger().info(str(left_motor_speed))
        self.get_logger().info(str(right_motor_speed))

        if left_motor_speed > 0:
            normalized_left_motor_speed = (left_motor_speed / MAX_MOTOR_ROTATION_SPEED) * 60 + 40
        elif left_motor_speed < 0:
            normalized_left_motor_speed = (left_motor_speed / MAX_MOTOR_ROTATION_SPEED) * 60 - 40
        else:
            normalized_left_motor_speed = 0

        if right_motor_speed > 0:
            normalized_right_motor_speed = (right_motor_speed / MAX_MOTOR_ROTATION_SPEED) * 60 + 40
        elif right_motor_speed < 0:
            normalized_right_motor_speed = (right_motor_speed / MAX_MOTOR_ROTATION_SPEED) * 60 - 40
        else:
            normalized_right_motor_speed = 0

        self.get_logger().info(str(normalized_left_motor_speed))
        self.get_logger().info(str(normalized_right_motor_speed))

        self.left_side_motor(normalized_left_motor_speed)
        self.right_side_motor(normalized_right_motor_speed)

    def obstacle_callback(self, msg: Bool):
        if msg.data:
            if not self.obstacle:
                self.stopMotors()
            self.obstacle = True
        else:
            self.obstacle = False 


def main(args=None):
    rclpy.init()
    node = DCMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()