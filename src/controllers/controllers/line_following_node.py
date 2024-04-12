import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from interfaces.msg import LineTracking
from std_srvs.srv import SetBool
from std_msgs.msg import String

from enum import Enum

class State(Enum):
    NO_LINE = "000"
    STEER_LEFT = "100"
    STEER_LIGHT_LEFT = "110"
    FORWARD_ALL = "111"
    FORWARD_MIDDLE = "010"
    STEER_LIGHT_RIGHT = "011"
    STEER_RIGHT = "001"
    UNALLOWED_STATE = "101"

class LineFollowingNode(Node):

    def __init__(self):
        super().__init__("line_following_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_right_away', rclpy.Parameter.Type.BOOL),
            ]
        )
        self.start_right_away = self.get_parameter('start_right_away').get_parameter_value().bool_value

        self.line_visibility_subscriber = self.create_subscription(LineTracking, "/line_visibility", self.line_visibility_callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_publisher = self.create_publisher(String, "/line_following_state", 10)
        self.toggle_service = self.create_service(SetBool, "/toggle_line_following", self.toggle_callback)

        self.timer = self.create_timer(0.01, self.fsm_next_state)
        if not self.start_right_away:
            self.timer.cancel()

        self.line_status = LineTracking()
        self.state = State.NO_LINE
        self.counter = 0
        self.last_direction = State.FORWARD_ALL
        self.last_direction_counter = 0

        self.get_logger().info("InitDone")

    #stopping and enabling
    def toggle_callback(self, request: SetBool, response: SetBool):
        if request.data:
            self.timer.cancel()
            self.timer = self.create_timer(0.01, self.fsm_next_state)
        else:
            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()

        response.success = True
        return response

    #getting sensor readings from other node
    def line_visibility_callback(self, msg: LineTracking):
        self.line_status.left = msg.left
        self.line_status.middle = msg.middle
        self.line_status.right = msg.right

    #deciding state for next step based on current sensor readings
    def fsm_next_state(self):
        status = str(int(self.line_status.left)) + str(int(self.line_status.middle)) + str(int(self.line_status.right))

        self.next_state = State(status)

        #what to do when lose line
        if self.next_state == State.NO_LINE:
            self.counter += 1
            
            #run for 5 seconds
            if self.counter < 500:
                self.next_state = self.state

                if self.state == State.FORWARD_ALL or self.state == State.FORWARD_MIDDLE:
                    self.next_state = self.last_direction

                if self.next_state == State.STEER_LIGHT_LEFT:
                    self.next_state = State.STEER_LEFT

                if self.next_state == State.STEER_LIGHT_RIGHT:
                    self.next_state = State.STEER_RIGHT
        else:
            self.counter = 0

        if self.next_state == State.UNALLOWED_STATE:
            self.next_state = self.state

        if self.next_state in [State.STEER_LEFT, State.STEER_LIGHT_LEFT, State.STEER_RIGHT, State.STEER_LIGHT_RIGHT]:
            self.last_direction_counter = 0
            self.last_direction = self.next_state
        else:
            self.last_direction_counter += 1
            if self.last_direction_counter == 25:
                self.last_direction = State.FORWARD_ALL
        
        self.fsm_step()

    #depenging on state move in certain direcion
    def fsm_step(self):
        self.state = self.next_state

        if self.state == State.NO_LINE: 
            self.state_publisher.publish(String(data = State.NO_LINE.name))
            
            msg = Twist()
            self.publisher.publish(msg)

        elif self.state == State.FORWARD_ALL or self.state == State.FORWARD_MIDDLE: 
            self.state_publisher.publish(String(data = self.state.name))
            
            msg = Twist()
            msg.linear.x = 0.2
            self.publisher.publish(msg)

        elif self.state == State.STEER_LIGHT_LEFT:
            self.state_publisher.publish(String(data = State.STEER_LIGHT_LEFT.name))
            
            msg = Twist()
            msg.linear.x = 0.15
            msg.angular.z = 1.256
            self.publisher.publish(msg)

        elif self.state == State.STEER_LEFT:
            self.state_publisher.publish(String(data = State.STEER_LEFT.name))
            
            msg = Twist()
            msg.linear.x = 0.11
            msg.angular.z = 3.14
            self.publisher.publish(msg)

        elif self.state == State.STEER_LIGHT_RIGHT:
            self.state_publisher.publish(String(data = State.STEER_LIGHT_RIGHT.name))
            
            msg = Twist()
            msg.linear.x = 0.15
            msg.angular.z = -1.57
            self.publisher.publish(msg)

        elif self.state == State.STEER_RIGHT:
            self.state_publisher.publish(String(data = State.STEER_RIGHT.name))
            
            msg = Twist()
            msg.linear.x = 0.11
            msg.angular.z = -3.14
            self.publisher.publish(msg)
        
        else:
            pass #unknown state

def main():
    rclpy.init()
    node = LineFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()