import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import Float64
from interfaces.action import Servo

import time

class SimulatorServo(Node):

    def __init__(self):
        super().__init__("simulator_servo")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ros_action', rclpy.Parameter.Type.STRING),
                ('gazebo_topic', rclpy.Parameter.Type.STRING)
            ]
        )
        self.ros_action = self.get_parameter('ros_action').get_parameter_value().string_value
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value

        self.action_server = ActionServer(self, Servo, self.ros_action, self.action_callback)
        self.publisher = self.create_publisher(Float64, self.gazebo_topic, 10)
        
        self.get_logger().info("InitDone")

    def action_callback(self, goal_handle):

        #look up
        if goal_handle.request.mode == 1: 
            self.in_dec_rement_servo(-1)

        #look down
        elif goal_handle.request.mode == 2: 
            self.in_dec_rement_servo(1)

        else:
            pass

        goal_handle.succeed()
        result = Servo.Result()
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
    node = SimulatorServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()