import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from interfaces.msg import UltrasonicDistance
from std_srvs.srv import SetBool
from std_msgs.msg import String

import random
import time

class WanderingNode(Node):

    def __init__(self):
        super().__init__("wandering_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_right_away', False),
            ]
        )

        self.srv = self.create_service(SetBool, "/toggle_wandering", self.toggle_callback)
        self.ultrasonic_subscriber = self.create_subscription(UltrasonicDistance, "/ultrasonic_distance", self.distance_callback, 10)
        self.obstacle_subscriber = self.create_subscription(Bool, "/obstacle_detected", self.obstacle_callback, 10)
        self.side_obstacle_subscriber = self.create_subscription(Int8, "/side_obstacle", self.side_obstacle_callback, 10)
        self.publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.state_publisher = self.create_publisher(String, "/wandering_state", 10)


        self.timer = self.create_timer(1, self.fsm_step)
        if not self.get_parameter('start_right_away').get_parameter_value().bool_value:
            self.timer.cancel()

        self.state = "START"
        self.scan_counter = 0
        self.distance = 0

        self.get_logger().info("InitDone")

    def toggle_callback(self, request: SetBool, response: SetBool):
        
        if request.data:
            self.state = "DRIVE"
            self.timer.cancel()
            self.timer = self.create_timer(0.1, self.fsm_step)
        else:
            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()

        response.success = True
        return response

    def distance_callback(self, msg: UltrasonicDistance):
        self.distance = msg.distance

    def obstacle_callback(self, msg: Bool):
        self.obstacle = msg.data
        if msg.data and self.state not in ["DRIVE", "OBSTACLE", "FIRST_SIDE_OBSTACLE", "SECOND_SIDE_OBSTACLE", "SMALL_OBSTACLE_DETECTED", "EVADE_LEFT", "RECENTER_LEFT", "RECENTER_RIGHT", "RESOLVE_SCAN_RESULT"]:
            self.state = "OBSTACLE"

    def side_obstacle_callback(self, msg_in: Int8):
        if self.state == "PAUSE": #current DRIVE next PAUSE
            if msg_in.data > 0:
                self.state = "SMALL_OBSTACLE_DETECTED"
                self.timer.cancel()
                self.fsm_step()
            else:
                pass

    def fsm_step(self):

        if self.state == "START":
            self.state_publisher.publish(String(data = "START"))

            if random.random() < 0.5:
                self.state = "DRIVE"
            else:
                 self.state = "DRIVE"
            self.fsm_step()

        elif self.state == "PAUSE":
            self.state_publisher.publish(String(data = "PAUSE"))
            self.state = "START"
            
            msg = Twist()
            self.publisher.publish(msg)
            
            self.timer.cancel()
            self.timer = self.create_timer(1, self.fsm_step)

        elif self.state == "TURN":
            self.state_publisher.publish(String(data = "TURN"))
            self.state = "DRIVE"
            speed = random.uniform(-0.785, 0.785)
            msg = Twist()
            msg.angular.z = speed
            self.publisher.publish(msg)

            self.timer.cancel()
            duration = random.uniform(0, 2)
            self.timer = self.create_timer(duration, self.fsm_step)

        elif self.state == "DRIVE":
            self.state_publisher.publish(String(data = "DRIVE"))
            self.state = "PAUSE"

            speed = random.uniform(0, 0.30695)
            speed = 0.15
            msg = Twist()
            msg.linear.x = speed
            self.publisher.publish(msg)

            self.timer.cancel()
            duration = random.uniform(0, 5)
            duration = 2
            self.timer = self.create_timer(duration, self.fsm_step)

        elif self.state == "OBSTACLE":
            self.state_publisher.publish(String(data = "OBSTACLE"))
            self.state = "FIRST_SIDE_OBSTACLE"

            self.timer.cancel()
            
            #reverse
            msg = Twist()
            msg.linear.x = -0.2
            self.publisher.publish(msg)
            time.sleep(0.5)

            #turn in random direction
            msg.linear.x = 0.0
            self.chance = random.random()
            
            if self.chance < 0.5:
                msg.angular.z = 3.14
            else:
                msg.angular.z = -3.14
            
            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(0.5, self.fsm_step)
        
        elif self.state == "FIRST_SIDE_OBSTACLE":
            self.state_publisher.publish(String(data = "FIRST_SIDE_OBSTACLE"))

            if self.obstacle:
                self.state = "SECOND_SIDE_OBSTACLE"
                msg = Twist()
                msg.angular.z = 3.14
                self.publisher.publish(msg)
                
                self.timer.cancel()
                self.timer = self.create_timer(1, self.fsm_step)
            else:
                self.state = "DRIVE"
                self.fsm_step()

        elif self.state == "SECOND_SIDE_OBSTACLE":
            self.state_publisher.publish(String(data = "SECOND_SIDE_OBSTACLE"))
            self.state = "DRIVE"
            if self.obstacle:
                msg = Twist()
                if self.chance < 0.5:
                    msg.angular.z = -3.14
                else:
                    msg.angular.z = 3.14
                
                self.publisher.publish(msg)
                
                self.timer.cancel()
                self.timer = self.create_timer(0.5, self.fsm_step)
            else:
                self.fsm_step()

        elif self.state == "SMALL_OBSTACLE_DETECTED":
            self.state_publisher.publish(String(data = "SMALL_OBSTACLE_DETECTED"))
            msg = Twist()
            msg.angular.z = 3.14
            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(0.25, self.fsm_step)    
            self.state = "SCAN_TURN_RESOLVE"

        elif self.state == "SCAN_TURN_RESOLVE":
            self.state_publisher.publish(String(data = "SMALL_OBSTACLE_DETECTED"))
            if self.scan_counter == 0:
                self.distance_array = []
                self.scan_counter += 1
                self.state = "SCAN_STOP"
                
                msg = Twist()
                self.publisher.publish(msg)
                self.timer.cancel()
                self.timer = self.create_timer(0, self.fsm_step)    

            elif self.scan_counter > 0 and self.scan_counter < 6:
                self.distance_array.append(self.distance)
                
                self.state = "SCAN_STOP"
                self.scan_counter += 1
                
                msg = Twist()
                msg.angular.z = -3.14
                self.publisher.publish(msg)
                self.timer.cancel()
                self.timer = self.create_timer(0.1, self.fsm_step)

            elif self.scan_counter == 6:
                self.distance_array.append(self.distance)
                self.scan_counter = 0

                self.timer.cancel()
                self.timer = self.create_timer(0, self.fsm_step)    
                self.state = "RESOLVE_SCAN_RESULT"

            else:
                pass #err
            
        elif self.state == "SCAN_STOP":
            self.state_publisher.publish(String(data = "SCAN_STOP"))

            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(0.15, self.fsm_step)    
            self.state = "SCAN_TURN_RESOLVE"
        
        elif self.state == "RESOLVE_SCAN_RESULT":
            self.state_publisher.publish(String(data = "RESOLVE_SCAN_RESULT"))

            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()

            left = min(self.distance_array[0], min(self.distance_array[1], self.distance_array[2]))
            right = min(self.distance_array[3], min(self.distance_array[4], self.distance_array[5]))

            if left < 0.7 and right > 0.7: #obstacle left
                self.state = "RECENTER_LEFT"
                self.timer = self.create_timer(0, self.fsm_step)
                
            elif right < 0.7 and left > 0.7: #obstacle right
                self.state = "EVADE_LEFT"
                self.timer = self.create_timer(0, self.fsm_step)

            else: #obstacle both or false alarm
                self.state = "DRIVE"
                
                msg = Twist()
                msg.angular.z = 3.14
                self.publisher.publish(msg)
                time.sleep(0.25)
                msg.angular.z = 0.0
                self.publisher.publish(msg)

                self.timer = self.create_timer(0, self.fsm_step)

        elif self.state == "EVADE_LEFT":
            self.state_publisher.publish(String(data = "EVADE_LEFT"))

            msg_out = Twist()
            msg_out.linear.x = 0.0
            msg_out.angular.z = 3.14
            self.publisher.publish(msg_out)
            self.timer.cancel()
            self.timer = self.create_timer(0.5, self.fsm_step)
            self.state = "RECENTER_RIGHT"

        elif self.state == "RECENTER_RIGHT":
            self.state_publisher.publish(String(data = "RECENTER_RIGHT"))
            msg_out = Twist()
            msg_out.linear.x = 0.2
            self.publisher.publish(msg_out)
            time.sleep(1)

            msg_out.linear.x = 0.15
            msg_out.angular.z = -0.785
            self.publisher.publish(msg_out)
            
            self.timer.cancel()
            self.timer = self.create_timer(1, self.fsm_step)
            self.state = "DRIVE"

        elif self.state == "RECENTER_LEFT":
            self.state_publisher.publish(String(data = "RECENTER_LEFT"))
            msg_out = Twist()
            msg_out.linear.x = 0.2
            self.publisher.publish(msg_out)
            time.sleep(1)

            msg_out.linear.x = 0.15
            msg_out.angular.z = 0.785
            self.publisher.publish(msg_out)

            self.timer.cancel()
            self.timer = self.create_timer(1, self.fsm_step)
            self.state = "DRIVE"

def main():
    rclpy.init()
    node = WanderingNode()
    rclpy.spin(node)
    rclpy.shutdown()