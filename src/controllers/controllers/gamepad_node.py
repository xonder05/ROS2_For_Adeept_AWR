import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from adeept_awr_interfaces.srv import RGB
from adeept_awr_interfaces.action import Servo

import pygame

class GamepadNode(Node):

    def __init__(self):
        super().__init__("gamepad_node")

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joysticks/controllers found.")
            self.destroy_node()
            return
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

        self.create_timer(0.01, self.pygame_loop)

        self.drive_axes = [0.0] * 2
        self.dpad_mode = 0
        
        self.rgb_mode = 0
        self.curr_color = [0] * 3

        #self.twist_subscriber = self.create_subscription(Twist, "/drive_directions", self.obstacle_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.led_color_client = self.create_client(RGB, "change_rgb_color")
        #while not self.led_color_client.wait_for_service(timeout_sec=1.0):
            #self.get_logger().info("led service not available waiting")
        
        self.action_client = ActionClient(self, Servo, 'put_servo_to_pos')

        self.get_logger().info("InitDone")


    def pygame_loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Exiting due to QUIT event.")
                self.destroy_node()

            elif event.type == pygame.JOYAXISMOTION:
                msg = Twist()

                if event.axis == 1: #linear
                    self.drive_axes[1] = -1 * event.value * 0.153475
                    
                if event.axis == 0: #angular
                    self.drive_axes[0] = -1 * event.value * 0.785

                msg.linear.x = self.drive_axes[1]
                msg.angular.z = self.drive_axes[0]
                self.twist_publisher.publish(msg)

            elif event.type == pygame.JOYBUTTONDOWN:
                button = event.button
                self.get_logger().info(f"Button {button} pressed")

            elif event.type == pygame.JOYHATMOTION:
                
                if self.dpad_mode == 0 and event.value[1] != 0: #servo
                    goal_msg = Servo.Goal()
                    goal_msg.value = 20

                    if event.value[1] == 1: #increment
                        goal_msg.mode = 1
                        self.get_logger().info(f"inc")

                    if event.value[1] == -1: #decrement
                        goal_msg.mode = 2
                        self.get_logger().info(f"dec")

                    #self.action_client.wait_for_server()
                    #self.goal_future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)
                    #self.goal_future.add_done_callback(self.response_callback)

                if self.dpad_mode == 1: #rbg
                    if event.value[0] != 0:
                        self.rgb_mode = (self.rgb_mode + 1) % 3

                    if event.value[1] == 1:
                        self.curr_color[self.rgb_mode] = min(self.curr_color[self.rgb_mode] + 32, 255)

                    if event.value[1] == -1:
                        self.curr_color[self.rgb_mode] = max(self.curr_color[self.rgb_mode] - 32, 0)

                    if event.value[1] != 0:
                        request = RGB.Request()
                        request.r = self.curr_color[0]
                        request.g = self.curr_color[1]
                        request.b = self.curr_color[2]
                    
                        self.get_logger().info(str(request.r))
                        self.get_logger().info(str(request.g))
                        self.get_logger().info(str(request.b))

                        #self.future = self.client.call_async(self.request)
                        #rclpy.spin_until_future_complete(self, self.future)

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
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.curr_position))






def main():
    rclpy.init()
    node = GamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()