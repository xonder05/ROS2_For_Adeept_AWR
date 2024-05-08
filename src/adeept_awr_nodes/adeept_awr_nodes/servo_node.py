import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from interfaces.action import Servo

import Adafruit_PCA9685
import time

class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_gen_channel', rclpy.Parameter.Type.INTEGER),
                ('rotation_limit_up', rclpy.Parameter.Type.INTEGER),
                ('rotation_limit_down', rclpy.Parameter.Type.INTEGER),
                ('default_position', rclpy.Parameter.Type.INTEGER),
            ]
        )
        self.pwm_gen_channel = self.get_parameter('pwm_gen_channel').get_parameter_value().integer_value
        self.rotation_limit_up = self.get_parameter('rotation_limit_up').get_parameter_value().integer_value
        self.rotation_limit_down = self.get_parameter('rotation_limit_down').get_parameter_value().integer_value
        self.default_position = self.get_parameter('default_position').get_parameter_value().integer_value
        
        self.action_server = ActionServer(self, Servo, '/put_servo_to_pos', self.callback)

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)
        self.curr_servo_pos = self.default_position

        self.get_logger().info('InitDone')

    def callback(self, goal_handle):
        feedback_msg = Servo.Feedback()

        #parse message to determine target position of the servo
        target_position = self.curr_servo_pos
        if goal_handle.request.mode == 0: #request contains absolute required position
            target_position = self.checkLimits(goal_handle.request.value)
    
        elif goal_handle.request.mode == 1: #request contains decrement (looking up) from current position
            target_position = self.checkLimits(self.curr_servo_pos - goal_handle.request.value)

        elif goal_handle.request.mode == 2: #request contains increment (looking down) from current position
            target_position = self.checkLimits(self.curr_servo_pos + goal_handle.request.value)

        #rotating servo and sending feedback
        for i in self.moveToPos(target_position):
            if i % 10 == 0:
                feedback_msg.curr_position = self.curr_servo_pos
                goal_handle.publish_feedback(feedback_msg)

        #target position reached
        goal_handle.succeed()
        result = Servo.Result()
        result.end_position = self.curr_servo_pos
        return result

    #limit the servo rotation angles so the camera does not break itself
    def checkLimits(self, target_position):
        if target_position > self.rotation_limit_down:
            return self.rotation_limit_down
        elif target_position < self.rotation_limit_up:
            return self.rotation_limit_up
        else:
            return target_position

    #continuous change in servo angle to the desired location
    def moveToPos(self, target_position):
        direction = 1 if target_position > self.curr_servo_pos else -1
        for _ in range(self.curr_servo_pos, target_position, direction):
            self.curr_servo_pos += direction
            self.pwm.set_pwm(self.pwm_gen_channel, 0, self.curr_servo_pos)
            yield self.curr_servo_pos
            time.sleep(0.05)

def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        for _ in node.moveToPos(node.default_position):
            pass
        node.destroy_node()
    finally:
        rclpy.shutdown()