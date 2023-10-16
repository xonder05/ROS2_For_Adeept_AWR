import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from adeept_awr_interfaces.action import Servo

import Adafruit_PCA9685 # Import the library used to communicate with PCA9685
import time



class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")
        self.action_server = ActionServer(self, Servo, 'put_servo_to_pos', self.execute_callback)

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)
        self.curr_servo_pos = 300


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Servo.Feedback()

        diff = self.curr_servo_pos - goal_handle.request.target_position

        if diff < 0:
            for i in range(0,abs(diff)):
                self.curr_servo_pos += 1
                self.pwm.set_pwm(0, 0, (self.curr_servo_pos))
                feedback_msg.curr_position = self.curr_servo_pos
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.05)
        else:
            for i in range(0,diff):
                self.curr_servo_pos -= 1
                self.pwm.set_pwm(0, 0, (self.curr_servo_pos))
                feedback_msg.curr_position = self.curr_servo_pos
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)

        goal_handle.succeed()

        result = Servo.Result()
        result.end_position = self.curr_servo_pos
        return result


    def moveServo(self, goal_pos):
        diff = self.curr_servo_pos - goal_pos

        if diff < 0:
            for i in range(0,diff):
                self.curr_servo_pos += 1
                self.pwm.set_pwm(3, 0, (self.curr_servo_pos))
                time.sleep(0.05)
        else:
            for i in range(0,diff):
                self.curr_servo_pos -= 1
                self.pwm.set_pwm(3, 0, (self.curr_servo_pos))
                time.sleep(0.05)


def main():
    rclpy.init()
    node = ServoNode()
    rclpy.spin(node)
    rclpy.shutdown()