import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from interfaces.action import Servo
from std_srvs.srv import SetBool

import pygame

class GamepadNode(Node):

    def __init__(self):
        super().__init__("gamepad_node")

        self.srv = self.create_service(SetBool, "/toggle_gamepad", self.toggle_callback)
        self.twist_publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.action_client = ActionClient(self, Servo, '/put_servo_to_pos')
        self.timer = self.create_timer(0.01, self.pygame_loop)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joysticks/controllers found.")
            self.destroy_node()
            return
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.drive_axes = [0.0] * 2

        self.get_logger().info("InitDone")

    def toggle_callback(self, request: SetBool, response: SetBool):
        self.get_logger().info("callback")

        if request.data:
            pygame.event.get()
            self.timer.cancel()
            self.timer = self.create_timer(0.01, self.pygame_loop)
        else:
            self.timer.cancel()

        response.success = True
        return response

    def pygame_loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Exiting due to QUIT event.")
                self.destroy_node()

            elif event.type == pygame.JOYAXISMOTION:
                msg = Twist()

                if abs(event.value) < 0.1:
                    event.value = 0

                if event.axis == 3: #linear
                    self.drive_axes[1] = -1 * event.value * 0.30695
                    
                if event.axis == 2: #angular
                    self.drive_axes[0] = -1 * event.value * 4.385

                msg.linear.x = self.drive_axes[1]
                msg.angular.z = self.drive_axes[0]
                self.twist_publisher.publish(msg)

            elif event.type == pygame.JOYHATMOTION:

                if event.value[1] != 0 and event.value[0] == 0: #servo
                    goal_msg = Servo.Goal()
                    goal_msg.value = 20

                    if event.value[1] == 1: #increment
                        goal_msg.mode = 1
                        self.get_logger().info(f"inc")

                    if event.value[1] == -1: #decrement
                        goal_msg.mode = 2
                        self.get_logger().info(f"dec")

                    self.action_client.wait_for_server()
                    self.goal_future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)
                    self.goal_future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.curr_position))


def main():
    rclpy.init()
    node = GamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()