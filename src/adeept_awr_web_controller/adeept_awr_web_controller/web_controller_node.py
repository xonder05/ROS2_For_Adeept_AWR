import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from adeept_awr_interfaces.srv import WebCommand
from adeept_awr_interfaces.action import Servo


class WebControllerNode(Node):
    def __init__(self):
        super().__init__("web_controller_node")
        self.srv = self.create_service(WebCommand, "web_controller", self.callback)
        
        self.action_client = ActionClient(self, Servo, 'put_servo_to_pos')
        self.get_logger().info("InitDone")

    def callback(self, request, response):
        if request.type == 5: #servo command
            if (request.value == 0):
                self.send_goal(1)
            else:
                self.send_goal(2)
        else:
            pass

        return response

    def send_goal(self, direction):
        goal_msg = Servo.Goal()
        goal_msg.mode = direction
        goal_msg.value = 20

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

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.curr_position))

    def result_callback(self, future):
        self.get_logger().info('Result: {0}'.format(future.result().result))

def main():
    rclpy.init()
    node = WebControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()