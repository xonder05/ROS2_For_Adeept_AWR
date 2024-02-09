import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import Float64
from adeept_awr_interfaces.action import Servo

import time

class SimulatorCameraServo(Node):

    def __init__(self):
        super().__init__("simulator_camera_servo")
        
        self.action_server = ActionServer(self, Servo, 'put_servo_to_pos', self.action_callback)
        self.publisher = self.create_publisher(Float64, "/model/adeept_awr/joint/camera_servo_joint/cmd_vel", 10)
        
        self.get_logger().info("InitDone")

    def action_callback(self, goal_handle):
        feedback_msg = Servo.Feedback()
        
        if goal_handle.request.mode == 1: # decrement (look up)
            self.in_dec_rement_servo(-1)

        elif goal_handle.request.mode == 2: # increment (look down)
            self.in_dec_rement_servo(1)

        else:
            pass

        goal_handle.succeed()
        result = Servo.Result()
        result.end_position = 200
        return result

    def in_dec_rement_servo(self, direction):
        msg = Float64()
        msg.data = 0.3925 * direction
        self.publisher.publish(msg)
        time.sleep(0.5)
        msg.data = 0.0
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SimulatorCameraServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()