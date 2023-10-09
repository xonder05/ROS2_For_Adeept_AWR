import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from adeept_awr_interfaces.action import Servo

class ServoAngleChangerNode(Node):

    def __init__(self):
        super().__init__('servo_angle_changer')
        self.action_client = ActionClient(self, Servo, 'put_servo_to_pos')

    def send_goal(self, target_position):
        goal_msg = Servo.Goal()
        goal_msg.target_position = target_position

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
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.curr_position))


def main(args=None):
    rclpy.init(args=args)

    node = ServoAngleChangerNode()

    while(1):
        try:
            user_input = int(input("Enter an integer: "))
            node.send_goal(user_input)

        except ValueError:
            pass
