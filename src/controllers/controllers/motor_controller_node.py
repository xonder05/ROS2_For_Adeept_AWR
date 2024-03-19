import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
from interfaces.action import MCC

import time

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__("motor_controller_node")

        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('maximum_angular_speed', rclpy.Parameter.Type.DOUBLE),
        #     ]
        # )
        # self.maximum_angular_speed = self.get_parameter('maximum_angular_speed').get_parameter_value().double_value

        self.subscriber = self.create_subscription(Twist, "/imu_node/sensor_reading", self.imu_callback, 10)
        self.action_server = ActionServer(self, MCC, '/motor_command',
                                        handle_accepted_callback=self.action_callback, execute_callback=self.execute_callback)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.current_angle = 0

        self.get_logger().info("InitDone")

    def imu_callback(self, msg):
        self.current_angle = msg.angular.z

    def action_callback(self, goal_handle):
        if goal_handle.request.command == 0:
            
            self.goal_angle = self.current_angle + goal_handle.request.value
            
            msg = Twist()
            if self.goal_angle < self.current_angle:
                msg.angular.z = 4.7
                self.direction = True 
            elif self.goal_angle > self.current_angle:
                msg.angular.z = -4.7
                self.direction = False
            else: return #did you just order me to turn zero degrees?
            self.publisher.publish(msg)

            self.goal_handle = goal_handle
            self.timer = self.create_timer(0.1, self.action_completion_callback)
            
    def action_completion_callback(self):
        if self.direction:
            if self.goal_angle < self.current_angle:
                self.get_logger().info(f"goal: {self.goal_angle}, angle: {self.current_angle}")
                return    
        else:
            if self.goal_angle > self.current_angle:
                self.get_logger().info(f"goal: {self.goal_angle}, angle: {self.current_angle}")
                return

        msg = Twist()
        self.publisher.publish(msg)

        self.timer.cancel()

        self.goal_handle.execute()

    def execute_callback(self, goal_handle):
        self.get_logger().info("suceed in execute")
        goal_handle.succeed()
        result = MCC.Result()
        return result

def main():
    rclpy.init()
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()