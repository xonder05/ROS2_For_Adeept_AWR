import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from interfaces.msg import LineTracking
from std_srvs.srv import SetBool
from std_msgs.msg import String

class LineFollowingNode(Node):

    def __init__(self):
        super().__init__("line_following_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_right_away', False),
            ]
        )

        self.state_publisher = self.create_publisher(String, "/line_tracking_state", 10)
        self.srv = self.create_service(SetBool, "/toggle_line_following", self.toggle_callback)
        self.line_visibility_subscriber = self.create_subscription(LineTracking, "/line_visibility", self.callback, 10)
        self.publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.state = ""
        self.next_state = ""
        self.line_status = LineTracking()
        
        self.timer = self.create_timer(0.05, self.fsm_next_state)
        if not self.get_parameter('start_right_away').get_parameter_value().bool_value:
            self.timer.cancel()
        
        self.get_logger().info("InitDone")

    def toggle_callback(self, request: SetBool, response: SetBool):
        if request.data:
            self.timer.cancel()
            self.timer = self.create_timer(0.05, self.fsm_next_state)
        else:
            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()

        response.success = True
        return response

    def callback(self, msg: LineTracking):
        self.line_status.left = msg.left
        self.line_status.middle = msg.middle
        self.line_status.right = msg.right

    def fsm_next_state(self):
        status = str(int(self.line_status.left)) + str(int(self.line_status.middle)) + str(int(self.line_status.right))
        print(status)

        if status == "000":
            self.next_state = "NO_LINE"
        
        elif status == "100":
            self.next_state = "STEER_LEFT"

        elif status == "110":
            self.next_state = "STEER_LIGHT_LEFT"

        elif status == "111" or status == "010":
            self.next_state = "FORWARD"

        elif status == "011":
            self.next_state = "STEER_LIGHT_RIGHT"

        elif status == "001":
            self.next_state = "STEER_RIGHT"

        else:
            print("next step error")

        self.fsm_step()

    def fsm_step(self):
        if self.state != self.next_state and self.state != "BREAK":
            self.state = "BREAK"
        else:
            self.state = self.next_state

        if self.state == "BREAK":
            self.state_publisher.publish(String(data = "BREAK"))
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)

        elif self.state == "NO_LINE": # 0 0 0
            self.state_publisher.publish(String(data = "NO_LINE"))
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)

        elif self.state == "FORWARD": # 0 1 0 / 1 1 1
            self.state_publisher.publish(String(data = "FORWARD"))
            msg = Twist()
            msg.linear.x = 0.15
            msg.angular.z = 0.0
            self.publisher.publish(msg)

        elif self.state == "STEER_LIGHT_LEFT": # 0 1 1
            self.state_publisher.publish(String(data = "STEER_LIGHT_LEFT"))
            msg = Twist()
            msg.linear.x = 0.1
            msg.angular.z = 1.0
            self.publisher.publish(msg)

        elif self.state == "STEER_LEFT": # 0 0 1
            self.state_publisher.publish(String(data = "STEER_LEFT"))
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 2.0
            self.publisher.publish(msg)

        elif self.state == "STEER_LIGHT_RIGHT": # 1 1 0
            self.state_publisher.publish(String(data = "STEER_LIGHT_RIGHT"))
            msg = Twist()
            msg.linear.x = 0.1
            msg.angular.z = -1.0
            self.publisher.publish(msg)

        elif self.state == "STEER_RIGHT": # 1 0 0
            self.state_publisher.publish(String(data = "STEER_RIGHT"))
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = -2.0
            self.publisher.publish(msg)
        
        else:
            pass #unknown state

def main():
    rclpy.init()
    node = LineFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        node.publisher.publish(msg)
    finally:
        rclpy.shutdown()

    