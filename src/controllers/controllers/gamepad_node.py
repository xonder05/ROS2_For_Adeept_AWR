import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from interfaces.action import Servo

import pygame

class GamepadNode(Node):

    def __init__(self):
        super().__init__("gamepad_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_right_away', rclpy.Parameter.Type.BOOL),
                ('maximum_linear_speed', rclpy.Parameter.Type.DOUBLE),
                ('maximum_angular_speed', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.start_right_away = self.get_parameter('start_right_away').get_parameter_value().bool_value
        self.maximum_linear_speed = self.get_parameter('maximum_linear_speed').get_parameter_value().double_value
        self.maximum_angular_speed = self.get_parameter('maximum_angular_speed').get_parameter_value().double_value

        self.service = self.create_service(SetBool, "/toggle_gamepad", self.toggle_callback)
        self.publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.servo_action_client = ActionClient(self, Servo, '/put_servo_to_pos')

        self.timer = self.create_timer(0.01, self.pygame_loop)
        if not self.start_right_away:
            self.timer.cancel()

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No controllers found, Exiting")
            self.destroy_node()
            return
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.drive_axes = [0.0] * 2

        self.get_logger().info("InitDone")

    #allows stoping of the event loop (used from user interface)
    def toggle_callback(self, request: SetBool, response: SetBool):
        if request.data:
            pygame.event.get()
            self.timer.cancel()
            self.timer = self.create_timer(0.01, self.pygame_loop)
        else:
            self.timer.cancel()

        response.success = True
        return response

    #main event loop for reading gamepad input
    def pygame_loop(self):
        for event in pygame.event.get():

            #joystics
            if event.type == pygame.JOYAXISMOTION:
                msg = Twist()

                #deadzone
                if abs(event.value) < 0.1:
                    event.value = 0

                #linear (right joystic forward and back)
                if event.axis == 3: 
                    self.drive_axes[1] = -1 * event.value * self.maximum_linear_speed
                
                #angular (right joystic left and right)
                if event.axis == 2: 
                    self.drive_axes[0] = -1 * event.value * self.maximum_angular_speed

                msg.linear.x = self.drive_axes[1]
                msg.angular.z = self.drive_axes[0]
                self.publisher.publish(msg)

            #dpad
            elif event.type == pygame.JOYHATMOTION:

                #servo (ignore left, right and combined directions)
                if event.value[1] != 0 and event.value[0] == 0: 
                    goal_msg = Servo.Goal()
                    goal_msg.value = 20

                    #increment (up)
                    if event.value[1] == 1: 
                        goal_msg.mode = 1

                    #decrement (down)
                    if event.value[1] == -1: 
                        goal_msg.mode = 2

                    self.servo_action_client.wait_for_server()
                    self.goal_future = self.servo_action_client.send_goal_async(goal_msg)
                    self.goal_future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Action server rejected request')
            return

def main():
    rclpy.init()
    node = GamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()