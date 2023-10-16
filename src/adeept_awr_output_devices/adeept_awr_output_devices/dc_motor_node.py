import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import DriveDirections

import time
import RPi.GPIO as GPIO

GPIO.cleanup()
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#todo adjust those values
Motor_A_EN = 4
Motor_B_EN = 17
Motor_A_Pin1 = 26
Motor_A_Pin2 = 21
Motor_B_Pin1 = 27
Motor_B_Pin2 = 18

class DCMotor():
    
    def motorStop(self):
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)


    def setup(self): # GPIO initialization, GPIO motor cannot be controlled without initialization
        global pwm_A, pwm_B
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Motor_A_EN, GPIO.OUT)
        GPIO.setup(Motor_B_EN, GPIO.OUT)
        GPIO.setup(Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(Motor_B_Pin2, GPIO.OUT)
        
        self.motorStop() # Avoid motor starting to rotate automatically after initialization
        
        try: # Try is used here to avoid errors due to repeated PWM settings
            pwm_A = GPIO.PWM(Motor_A_EN, 1000)
            pwm_B = GPIO.PWM(Motor_B_EN, 1000)
        except:
            pass


    def motor_A(self, direction, speed): # The function used to control the motor of port A
        if direction == 1:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        if direction == -1:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)


    def motor_B(self, direction, speed): # The function used to control the motor of port B
        if direction == 1:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        if direction == -1:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)



class DCMotorNode(Node):

    def __init__(self):
        super().__init__("dc_motor_node")
        self.subscriber = self.create_subscription(DriveDirections, "drive_directions", self.callback, 10)
        self.motor = DCMotor()
        self.motor.setup()

    def callback(self, msg: DriveDirections):
        self.get_logger().info(chr(msg.direction))

        direction = chr(msg.direction)
        if (direction == "w"):
            self.motor.motor_A(1, 100)
            self.motor.motor_B(1, 100)
            time.sleep(1)
            self.motor.motorStop()
        elif (direction == "s"):
            self.motor.motor_A(-1, 100)
            self.motor.motor_B(-1, 100)
            time.sleep(1)
            self.motor.motorStop()
        elif (direction == "a"):
            pass
        elif (direction == "d"):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DCMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()