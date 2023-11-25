import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from adeept_awr_interfaces.srv import WebCommand
from adeept_awr_interfaces.msg import DriveDirections
from adeept_awr_interfaces.action import Servo


class WebControllerNode(Node):
    def __init__(self):
        super().__init__("web_controller_node")
        self.srv = self.create_service(WebCommand, "web_controller", self.callback)
        
        self.publisher = self.create_publisher(DriveDirections, "drive_directions", 10)
        self.action_client = ActionClient(self, Servo, 'put_servo_to_pos')
        print("ready")

    def callback(self, request, response):
        if request.type == 0: #drive direction start
            self.direction = request.value
            self.drive_timer = self.create_timer(0.5, self.drive_direction_publisher)
        elif request.type == 1: #drive direction stop
            self.drive_timer.cancel()
        elif request.type == 5: #servo command
            if (request.value == 0):
                self.send_goal(300)
            else:
                self.send_goal(250)
        else:
            pass
        return response

    def drive_direction_publisher(self):
        output = DriveDirections()
        output.direction = self.direction
        self.publisher.publish(output)

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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.curr_position))

def main():
    rclpy.init()
    node = WebControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()