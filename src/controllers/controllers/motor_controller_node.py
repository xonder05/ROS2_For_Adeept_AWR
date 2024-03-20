import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse

from geometry_msgs.msg import Twist
from interfaces.action import MCC

import threading

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__("motor_controller_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('base_turning_speed', rclpy.Parameter.Type.DOUBLE),
                ('imu_topic', rclpy.Parameter.Type.STRING),
                ('commands_topic', rclpy.Parameter.Type.STRING),
            ]
        )
        self.base_turning_speed = self.get_parameter('base_turning_speed').get_parameter_value().double_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.commands_topic = self.get_parameter('commands_topic').get_parameter_value().string_value

        self.subscriber = self.create_subscription(Twist, self.imu_topic, self.imu_callback, 10)
        self.action_server = ActionServer(self, MCC, self.commands_topic,
                                            goal_callback=self.goal_callback,
                                            handle_accepted_callback=self.handle_accepted_callback,
                                            execute_callback=self.execute_callback)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.current_angle = 0
        self.goal_handle = None

        self.get_logger().info("InitDone")

    #getting imu readings
    def imu_callback(self, msg):
        self.current_angle = msg.angular.z

        if self.goal_handle is not None and self.goal_handle.is_active:
            if self.direction:
                if self.goal_angle >= self.current_angle:
                    self.goal_handle.execute()
            else:
                if self.goal_angle <= self.current_angle:
                    self.goal_handle.execute()

    #only one action at a time, two nodes should not control the motor simultaneously
    def goal_callback(self, goal_handle):
        if self.goal_handle is not None and self.goal_handle.is_active:
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT

    #calculate target angle and start turning
    def handle_accepted_callback(self, goal_handle):
        self.goal_angle = self.current_angle + (goal_handle.request.angle * 0.95)
        
        msg = Twist()
        if self.goal_angle < self.current_angle:
            msg.angular.z = self.base_turning_speed
            self.direction = True 
        elif self.goal_angle > self.current_angle:
            msg.angular.z = -self.base_turning_speed
            self.direction = False
        else: return
        self.publisher.publish(msg)

        self.goal_handle = goal_handle
    
    #target angle reached
    def execute_callback(self, goal_handle):
        msg = Twist()
        self.publisher.publish(msg)
        
        goal_handle.succeed()
        result = MCC.Result()
        return result

def main():
    rclpy.init()
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()