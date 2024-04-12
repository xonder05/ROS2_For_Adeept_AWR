import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from interfaces.action import MCC

from enum import Enum
import time
import random

class State(Enum):
    PAUSE = 1
    DRIVE = 2
    OBSTACLE = 3
    CHECK_FIRST_SIDE = 4
    CHECK_SECOND_SIDE = 5
    SCAN_START = 6
    SCAN_LOG_DISTANCE_AND_TURN = 7
    SCAN_WAIT_FOR_DISTANCE_READING = 8
    SCAN_RESOLVE_RESULTS = 9
    TURN_LEFT = 10

class WanderingNode(Node):

    def __init__(self):
        super().__init__("wandering_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_right_away', rclpy.Parameter.Type.BOOL),
            ]
        )
        self.start_right_away = self.get_parameter('start_right_away').get_parameter_value().bool_value

        self.ultrasonic_subscriber = self.create_subscription(Float32, "/ultrasonic_distance", self.distance_callback, 10)
        self.obstacle_subscriber = self.create_subscription(Bool, "/ultrasonic_obstacle_warning", self.obstacle_callback, 10)
        self.side_obstacle_subscriber = self.create_subscription(Bool, "/ultrasonic_obstacle_disappearance_warning", self.side_obstacle_callback, 10)
        self.not_moving_subscriber = self.create_subscription(Bool, "/not_moving_warning", self.obstacle_callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_publisher = self.create_publisher(String, "/wandering_state", 10)
        self.toggle_service = self.create_service(SetBool, "/toggle_wandering", self.toggle_callback)
        self.motor_controller_client = ActionClient(self, MCC, '/motor_command')

        self.timer = self.create_timer(1, self.fsm_step)
        if not self.start_right_away:
            self.timer.cancel()

        self.state = State.DRIVE
        self.cautious_mode = True
        self.scan_counter = 0
        self.distance = 0
        self.goal_handle = None

        self.get_logger().info("InitDone")

    #toggles fsm execution
    def toggle_callback(self, request: SetBool, response):
        if request.data:
            self.state = State.DRIVE
            self.timer.cancel()
            self.timer = self.create_timer(0.1, self.fsm_step)
        else:
            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()

        response.success = True
        return response

    def distance_callback(self, msg: Float32):
        self.distance = msg.data

    def obstacle_callback(self, msg: Bool):
        self.obstacle = msg.data
        
        #only react when going forward
        if msg.data and ( self.state == State.PAUSE or self.state == State.SCAN_START or self.state == State.SCAN_LOG_DISTANCE_AND_TURN or self.state == State.SCAN_WAIT_FOR_DISTANCE_READING):
            self.state = State.OBSTACLE
            self.timer.cancel()

            if self.goal_handle is not None and self.controller_state == "working":
                self.cancel_future = self.goal_handle.cancel_goal_async()
                self.cancel_future.add_done_callback(self.obstacle_second_part)
            else:
                self.timer = self.create_timer(0, self.fsm_step)

    def obstacle_second_part(self, future):
        handle = future.result()
        if len(handle.goals_canceling) > 0:
            self.timer = self.create_timer(0, self.fsm_step)
        else:
            pass

    def side_obstacle_callback(self, msg: Bool):
        #only react when going forward
        if msg.data and ( self.state == State.PAUSE or self.state == State.SCAN_START ):
            self.state = State.SCAN_START
            self.timer.cancel()

            if self.goal_handle is not None and self.controller_state == "working":
                self.cancel_future = self.goal_handle.cancel_goal_async()
                self.cancel_future.add_done_callback(self.obstacle_second_part)
            else:
                self.timer = self.create_timer(0, self.fsm_step)

    def call_motor_controller(self, angle):
        self.timer.cancel()
        self.motor_controller_client.wait_for_server()

        msg = MCC.Goal()
        msg.angle = angle
        
        future = self.motor_controller_client.send_goal_async(msg)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.controller_state = "working"
            future = self.goal_handle.get_result_async()
            future.add_done_callback(self.done_callback)
        else:
            pass

    def done_callback(self, future):
        result = future.result()
        self.controller_state = "done"
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.timer = self.create_timer(0, self.fsm_step)

    def fsm_step(self):
        #default state, causes robot to stop and after a while transition to DRIVE
        if self.state == State.PAUSE:
            self.state_publisher.publish(String(data = State.PAUSE.name))
            self.timer.cancel()
            
            msg = Twist()
            self.publisher.publish(msg)

            self.state = State.DRIVE            
            self.timer = self.create_timer(0.5, self.fsm_step)

        #just drive forward for random time interval
        elif self.state == State.DRIVE:
            self.state_publisher.publish(String(data = State.DRIVE.name))
            self.timer.cancel()
            
            msg = Twist()
            msg.linear.x = 0.5
            self.publisher.publish(msg)

            if self.cautious_mode:
                self.state = State.SCAN_START
            else:    
                self.state = State.PAUSE

            duration = random.uniform(1, 5)
            self.timer = self.create_timer(duration, self.fsm_step)

        #transition to this state is triggered by event
        elif self.state == State.OBSTACLE:
            self.state_publisher.publish(String(data = State.OBSTACLE.name))
            self.timer.cancel()
            
            #reverse
            msg = Twist()
            msg.linear.x = -0.5
            self.publisher.publish(msg)
            time.sleep(0.5)

            #turn approx 90 degrees in random direction
            self.chance = random.random()

            if self.chance < 0.5:
                self.call_motor_controller(1.57)
            else:
                self.call_motor_controller(-1.57)
            
            self.state = State.CHECK_FIRST_SIDE
        
        elif self.state == State.CHECK_FIRST_SIDE:
            self.state_publisher.publish(String(data = State.CHECK_FIRST_SIDE.name))
            self.timer.cancel()

            #obstacle - turn approx 180 degrees
            if self.obstacle:

                self.call_motor_controller(3.14)

                self.state = State.CHECK_SECOND_SIDE
            
            else: #free - drive forward
                self.state = State.PAUSE
                self.timer = self.create_timer(0, self.fsm_step)

        elif self.state == State.CHECK_SECOND_SIDE:
            self.state_publisher.publish(String(data = State.CHECK_SECOND_SIDE.name))
            self.timer.cancel()
        
            #obstacle - turn approx 90 degrees to drive back where you came from
            if self.obstacle:
                
                if self.chance < 0.5:
                    self.call_motor_controller(-1.57)
                else:
                    self.call_motor_controller(1.57)
                
                self.state = State.PAUSE
            
            else: #free - drive forward
                self.state = State.PAUSE
                self.timer = self.create_timer(0, self.fsm_step)

        #sequence of states that can for obstacles in approx -45 to +45 degress in front of robot
        elif self.state == State.SCAN_START:
            self.state_publisher.publish(String(data = State.SCAN_START.name))
            self.timer.cancel()

            self.scan_counter = 0
            self.call_motor_controller(0.785)

            self.state = State.SCAN_LOG_DISTANCE_AND_TURN

        elif self.state == State.SCAN_LOG_DISTANCE_AND_TURN:
            self.state_publisher.publish(String(data = State.SCAN_LOG_DISTANCE_AND_TURN.name))
            self.timer.cancel()
            
            #init empty array and without turning go to next state
            if self.scan_counter == 0:
                self.distance_array = []
                self.scan_counter += 1
                
                self.state = State.SCAN_WAIT_FOR_DISTANCE_READING
                self.timer = self.create_timer(0, self.fsm_step)    

            #save distance and start turning
            elif self.scan_counter > 0 and self.scan_counter < 6:
                self.distance_array.append(self.distance)
                self.scan_counter += 1
                
                self.call_motor_controller(-0.314)

                self.state = State.SCAN_WAIT_FOR_DISTANCE_READING

            #save last distance reading
            elif self.scan_counter == 6:
                self.distance_array.append(self.distance)
                self.scan_counter = 0

                self.state = State.SCAN_RESOLVE_RESULTS
                self.timer = self.create_timer(0, self.fsm_step)    

            else:
                pass #err
        
        #stops turning and starts waiting for distance reading from sensor
        elif self.state == State.SCAN_WAIT_FOR_DISTANCE_READING:
            self.state_publisher.publish(String(data = State.SCAN_WAIT_FOR_DISTANCE_READING.name))
            self.timer.cancel()

            self.state = State.SCAN_LOG_DISTANCE_AND_TURN
            self.timer = self.create_timer(0.15, self.fsm_step)    
        
        #resolves results from scannig
        elif self.state == State.SCAN_RESOLVE_RESULTS:
            self.state_publisher.publish(String(data = State.SCAN_RESOLVE_RESULTS.name))
            self.timer.cancel()

            left = min(self.distance_array[0], min(self.distance_array[1], self.distance_array[2]))
            right = min(self.distance_array[3], min(self.distance_array[4], self.distance_array[5]))

            #obstacle left - already pointed right can continue driving
            if left < 0.7 and right > 0.7:
                self.state = State.DRIVE
                self.timer = self.create_timer(0, self.fsm_step)
            
            #obstacle right - has to turn left first
            elif right < 0.7 and left > 0.7: 
                self.state = State.TURN_LEFT
                self.timer = self.create_timer(0, self.fsm_step)

            else: #obstacle both or false alarm, turn forward and continue
                self.call_motor_controller(0.785)

                self.state = State.PAUSE

        elif self.state == State.TURN_LEFT:
            self.state_publisher.publish(String(data = State.TURN_LEFT.name))
            self.timer.cancel()

            self.call_motor_controller(1.57)

            self.state = State.DRIVE

def main():
    rclpy.init()
    node = WanderingNode()
    rclpy.spin(node)
    rclpy.shutdown()