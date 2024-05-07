import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from interfaces.srv import SetFloat32

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
    SCAN_STOP_AND_WAIT_FOR_DISTANCE_READING = 8
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
        self.publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.state_publisher = self.create_publisher(String, "/wandering_state", 10)
        self.toggle_service = self.create_service(SetBool, "/toggle_wandering", self.toggle_callback)
        self.multiplier_service = self.create_service(SetFloat32, "/set_multiplier", self.multiplier_callback)

        self.timer = self.create_timer(1, self.fsm_step)
        if not self.start_right_away:
            self.timer.cancel()

        self.state = State.DRIVE
        self.cautious_mode = True
        self.scan_counter = 0
        self.distance = 0
        self.base_linear_speed = 0.2 #m/s
        self.base_angular_speed = 3.14 #rad/s
        self.multiplier = 0

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

    #sets speed multiplier (used for adeept which needs more power to get moving)
    def multiplier_callback(self, request: SetFloat32, response):
        self.multiplier = request.data
        response.success = True
        return response

    def distance_callback(self, msg: Float32):
        self.distance = msg.data

    def obstacle_callback(self, msg: Bool):
        self.obstacle = msg.data
        
        #only react when going forward
        if msg.data and ( self.state == State.PAUSE or self.state == State.SCAN_START ):
            self.state = State.OBSTACLE
            self.timer.cancel()
            self.timer = self.create_timer(0, self.fsm_step)

    def side_obstacle_callback(self, msg: Bool):
        
        #only react when going forward
        if msg.data and ( self.state == State.PAUSE or self.state == State.SCAN_START ):
            self.state = State.SCAN_START
            self.timer.cancel()
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
            msg.linear.x = self.base_linear_speed if self.multiplier == 0 else self.base_linear_speed * self.multiplier
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
            msg.linear.x = -self.base_linear_speed if self.multiplier == 0 else -self.base_linear_speed * self.multiplier
            self.publisher.publish(msg)
            time.sleep(0.5)

            #turn approx 90 degrees in random direction
            msg = Twist()
            self.chance = random.random()

            if self.chance < 0.5:
                msg.angular.z = self.base_angular_speed if self.multiplier == 0 else self.base_angular_speed * self.multiplier
            else:
                msg.angular.z = -self.base_angular_speed if self.multiplier == 0 else -self.base_angular_speed * self.multiplier
            
            self.publisher.publish(msg)
            
            self.state = State.CHECK_FIRST_SIDE
            self.timer = self.create_timer(0.5, self.fsm_step)
        
        elif self.state == State.CHECK_FIRST_SIDE:
            self.state_publisher.publish(String(data = State.CHECK_FIRST_SIDE.name))
            self.timer.cancel()

            #obstacle - turn approx 180 degrees
            if self.obstacle:

                msg = Twist()
                msg.angular.z = self.base_angular_speed if self.multiplier == 0 else self.base_angular_speed * self.multiplier
                self.publisher.publish(msg)

                self.state = State.CHECK_SECOND_SIDE
                self.timer = self.create_timer(1, self.fsm_step)
            
            else: #free - drive forward
                self.state = State.PAUSE
                self.timer = self.create_timer(0, self.fsm_step)

        elif self.state == State.CHECK_SECOND_SIDE:
            self.state_publisher.publish(String(data = State.CHECK_SECOND_SIDE.name))
            self.timer.cancel()
        
            #obstacle - turn approx 90 degrees to drive back where you came from
            if self.obstacle:
                
                msg = Twist()
                if self.chance < 0.5:
                    msg.angular.z = -self.base_angular_speed if self.multiplier == 0 else -self.base_angular_speed * self.multiplier
                else:
                    msg.angular.z = self.base_angular_speed if self.multiplier == 0 else self.base_angular_speed * self.multiplier
                
                self.publisher.publish(msg)

                self.state = State.PAUSE
                self.timer = self.create_timer(0.5, self.fsm_step)
            
            else: #free - drive forward
                self.state = State.PAUSE
                self.timer = self.create_timer(0, self.fsm_step)

        #sequence of states that can for obstacles in approx -45 to +45 degress in front of robot
        elif self.state == State.SCAN_START:
            self.state_publisher.publish(String(data = State.SCAN_START.name))
            self.timer.cancel()

            msg = Twist()
            msg.angular.z = self.base_angular_speed if self.multiplier == 0 else self.base_angular_speed * self.multiplier
            self.publisher.publish(msg)

            self.state = State.SCAN_LOG_DISTANCE_AND_TURN
            self.timer = self.create_timer(0.25, self.fsm_step)    

        elif self.state == State.SCAN_LOG_DISTANCE_AND_TURN:
            self.state_publisher.publish(String(data = State.SCAN_LOG_DISTANCE_AND_TURN.name))
            self.timer.cancel()
            
            #init empty array and without turning go to next state
            if self.scan_counter == 0:
                self.distance_array = []
                self.scan_counter += 1
                
                msg = Twist()
                self.publisher.publish(msg)
                
                self.state = State.SCAN_STOP_AND_WAIT_FOR_DISTANCE_READING
                self.timer = self.create_timer(0, self.fsm_step)    

            #save distance and start turning
            elif self.scan_counter > 0 and self.scan_counter < 6:
                self.distance_array.append(self.distance)
                self.scan_counter += 1
                
                msg = Twist()
                msg.angular.z = -self.base_angular_speed if self.multiplier == 0 else -self.base_angular_speed * self.multiplier * 2
                self.publisher.publish(msg)

                self.state = State.SCAN_STOP_AND_WAIT_FOR_DISTANCE_READING
                self.timer = self.create_timer(0.1, self.fsm_step)

            #save last distance reading
            elif self.scan_counter == 6:
                self.distance_array.append(self.distance)
                self.scan_counter = 0

                self.state = State.SCAN_RESOLVE_RESULTS
                self.timer = self.create_timer(0, self.fsm_step)    

            else:
                pass #err
        
        #stops turning and starts waiting for distance reading from sensor
        elif self.state == State.SCAN_STOP_AND_WAIT_FOR_DISTANCE_READING:
            self.state_publisher.publish(String(data = State.SCAN_STOP_AND_WAIT_FOR_DISTANCE_READING.name))
            self.timer.cancel()

            msg = Twist()
            self.publisher.publish(msg)

            self.state = State.SCAN_LOG_DISTANCE_AND_TURN
            self.timer = self.create_timer(0.15, self.fsm_step)    
        
        #resolves results from scannig
        elif self.state == State.SCAN_RESOLVE_RESULTS:
            self.state_publisher.publish(String(data = State.SCAN_RESOLVE_RESULTS.name))
            self.timer.cancel()

            msg = Twist()
            self.publisher.publish(msg)

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
                msg = Twist()
                msg.angular.z = self.base_angular_speed if self.multiplier == 0 else self.base_angular_speed * self.multiplier
                self.publisher.publish(msg)
                
                time.sleep(0.25)
                
                msg = Twist()
                self.publisher.publish(msg)

                self.state = State.PAUSE
                self.timer = self.create_timer(0, self.fsm_step)

        elif self.state == State.TURN_LEFT:
            self.state_publisher.publish(String(data = State.TURN_LEFT.name))
            self.timer.cancel()

            msg_out = Twist()
            msg_out.angular.z = self.base_angular_speed if self.multiplier == 0 else self.base_angular_speed * self.multiplier
            self.publisher.publish(msg_out)

            self.timer = self.create_timer(0.5, self.fsm_step)
            self.state = State.DRIVE

def main():
    rclpy.init()
    node = WanderingNode()
    rclpy.spin(node)
    rclpy.shutdown()