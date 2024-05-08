import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from interfaces.action import Servo

from pynput import keyboard

class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_right_away', rclpy.Parameter.Type.BOOL),
                ('linear_speed', rclpy.Parameter.Type.DOUBLE),
                ('angular_speed', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.start_right_away = self.get_parameter('start_right_away').get_parameter_value().bool_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        self.service = self.create_service(SetBool, "/toggle_keyboard", self.toggle_callback)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.servo_action_client = ActionClient(self, Servo, '/put_servo_to_pos')

        self.listener = keyboard.Listener(on_press=self.key_press_callback,on_release=self.key_release_callback)
        if self.start_right_away:
            self.listener.start()

        self.linear = 0.0
        self.angular = 0.0

        self.get_logger().info("InitDone")

    #allows stoping of the event loop (used from user interface)
    def toggle_callback(self, request: SetBool, response: SetBool):
        if request.data:
            self.listener.stop()
            self.listener = keyboard.Listener(on_press=self.key_press_callback,on_release=self.key_release_callback)
            self.listener.start()
        else:
            msg = Twist()
            self.publisher.publish(msg)
            self.listener.stop()

        response.success = True
        return response

    #callback function for keyboard listener
    def key_press_callback(self, key):
        try:
            if (key.char == "w"):
                self.linear = self.linear_speed
                self.publish_twist()

            elif (key.char == "s"):
                self.linear = -self.linear_speed
                self.publish_twist()

            elif (key.char == "a"):
                self.angular = self.angular_speed
                self.publish_twist()
            
            elif (key.char == "d"):
                self.angular = -self.angular_speed
                self.publish_twist()
            
            elif (key.char == "j"):
                self.send_goal(1)

            elif (key.char == "k"):
                self.send_goal(2)

            else:
                pass

        except AttributeError: pass

    #callback function for keyboard listener
    def key_release_callback(self, key):
        try:
            if (key.char == "w" or key.char == "s"):
                self.linear = 0.0
                self.publish_twist()

            elif (key.char == "a" or key.char == "d"):
                self.angular = 0.0
                self.publish_twist()

        except AttributeError: pass

    #to remove redundance
    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)

    #sending request to action server
    def send_goal(self, mode):
        goal_msg = Servo.Goal()
        goal_msg.value = 20
        goal_msg.mode = mode

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
    node = KeyboardNode()
    rclpy.spin(node)
    rclpy.shutdown()