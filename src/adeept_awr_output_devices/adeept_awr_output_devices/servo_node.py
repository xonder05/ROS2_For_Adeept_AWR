import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from adeept_awr_interfaces.action import Servo

import Adafruit_PCA9685
import time

class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")
        self.action_server = ActionServer(self, Servo, 'put_servo_to_pos', self.action_callback)

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)
        self.curr_servo_pos = 300

        self.get_logger().info('InitDone')


    def action_callback(self, goal_handle):
        feedback_msg = Servo.Feedback()

        if goal_handle.request.mode == 0: #absolute pos
            target_position = self.checkLimits(goal_handle.request.value)

            if self.curr_servo_pos < target_position:
                for value in self.moveToPosUp(target_position):
                    print(f"Received value: {value}")
                    feedback_msg.curr_position = self.curr_servo_pos
                    goal_handle.publish_feedback(feedback_msg)
            else:
                for value in self.moveToPosDown(target_position):
                    print(f"Received value: {value}")
                    feedback_msg.curr_position = self.curr_servo_pos
                    goal_handle.publish_feedback(feedback_msg)
        
        elif goal_handle.request.mode == 1: # decrement (look up)
            target_position = self.checkLimits(self.curr_servo_pos - goal_handle.request.value)
            for value in self.moveToPosDown(target_position):
                print(f"Received value: {value}")
                feedback_msg.curr_position = self.curr_servo_pos
                goal_handle.publish_feedback(feedback_msg)

        elif goal_handle.request.mode == 2: # increment (look down)
            target_position = self.checkLimits(self.curr_servo_pos + goal_handle.request.value)
            for value in self.moveToPosUp(target_position):
                print(f"Received value: {value}")
                feedback_msg.curr_position = self.curr_servo_pos
                goal_handle.publish_feedback(feedback_msg)

        else:
            pass

        goal_handle.succeed()
        result = Servo.Result()
        result.end_position = self.curr_servo_pos
        return result

    def checkLimits(self, target_position):
        upper_limit = 150 #almost vertical
        lower_limit = 350 #little below horizontal

        if target_position > lower_limit:
            return lower_limit
        elif target_position < upper_limit:
            return upper_limit
        else:
            return target_position

    def moveToPosUp(self, target_position):
        for i in range(self.curr_servo_pos, target_position):
            self.curr_servo_pos += 1
            self.pwm.set_pwm(0, 0, (self.curr_servo_pos))
            yield self.curr_servo_pos
            time.sleep(0.05)

    def moveToPosDown(self, target_position):
        for i in range(self.curr_servo_pos, target_position, -1):
            self.curr_servo_pos -= 1
            self.pwm.set_pwm(0, 0, (self.curr_servo_pos))
            yield self.curr_servo_pos
            time.sleep(0.05)

    def moveToDefaultPosition(self):
        defaultServoPosition = 300
        if self.curr_servo_pos < defaultServoPosition:
            for value in self.moveToPosUp(defaultServoPosition):
                pass
        else:
            for value in self.moveToPosDown(defaultServoPosition):
                pass

def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.moveToDefaultPosition()
        node.destroy_node()
        #rclpy.shutdown()
    finally:
        pass