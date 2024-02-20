import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from adeept_awr_interfaces.srv import RGB
from adeept_awr_interfaces.action import Servo
from std_srvs.srv import SetBool

from pynput import keyboard


class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")

        self.twist_publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.action_client = ActionClient(self, Servo, '/put_servo_to_pos')

        self.linear = 0.0
        self.angular = 0.0

        self.get_logger().info("InitDone")

        with keyboard.Listener(on_press=self.key_press_callback,on_release=self.key_release_callback) as listener:
            listener.join()

    def key_press_callback(self, key):
        try:
            print(f'Key {key.char} pressed')

            if (key.char == "w"):
                self.linear = 0.3
                self.publish_twist()

            elif (key.char == "s"):
                self.linear = -0.3
                self.publish_twist()

            elif (key.char == "a"):
                self.angular = 1.5
                self.publish_twist()
            
            elif (key.char == "d"):
                self.angular = -1.5
                self.publish_twist()
            
            elif (key.char == "j"):
                self.send_goal(1)
            elif (key.char == "k"):
                self.send_goal(2)
            else:
                pass

        except AttributeError:
            print(f'Special key {key} pressed')

    def key_release_callback(self, key):
        try:
            print(f'Key {key} released')
        
            if (key.char == "w" or key.char == "s"):
                self.linear = 0.0
                self.publish_twist()

            elif (key.char == "a" or key.char == "d"):
                self.angular = 0.0
                self.publish_twist()

        except AttributeError:
            print(f'Special key {key} released')
        
    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.twist_publisher.publish(msg)

    def send_goal(self, mode):
        goal_msg = Servo.Goal()
        goal_msg.value = 20
        goal_msg.mode = mode

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
    node = KeyboardNode()
    rclpy.spin(node)
    rclpy.shutdown()