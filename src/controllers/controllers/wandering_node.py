import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from adeept_awr_interfaces.msg import UltrasonicDistance

import random
import time

class WanderingNode(Node):

    def __init__(self):
        super().__init__("wandering_node")
        self.obstacle_subscriber = self.create_subscription(Bool, "/obstacle_detected", self.obstacle_callback, 10)
        self.side_obstacle_subscriber = self.create_subscription(Int8, "/side_obstacle", self.side_obstacle_callback, 10)
        self.ultrasonic_subscriber = self.create_subscription(UltrasonicDistance, "/ultrasonic_distance", self.distance_callback, 10)
        self.publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.state = "START"
        self.timer = self.create_timer(1, self.fsm_step)
        self.scan_counter = 0
        self.distance_array = [0] * 5
        self.distance_index = 0
        self.distance = 0

        self.get_logger().info("InitDone")
        self.fsm_step()

    def obstacle_callback(self, msg: Bool):
        self.obstacle = msg.data
        if msg.data and self.state not in ["DRIVE", "OBSTACLE", "FIRST_SIDE_OBSTACLE", "SECOND_SIDE_OBSTACLE", "SMALL_OBSTACLE_DETECTED", "CHECK_LIGHT_LEFT", "CHECK_LIGHT_RIGHT", "EVADE_LEFT", "EVADE_RIGHT", "RECENTER_LEFT", "RECENTER_RIGHT", "SCAN_FOR_OBSTACLES", "RESOLVE_SCAN_RESULT"]:
            self.state = "OBSTACLE"

    def distance_callback(self, msg: UltrasonicDistance):
        self.distance = msg.distance
        self.distance_array[self.distance_index] = msg.distance
        self.distance_index = (self.distance_index + 1) % 5 

    def side_obstacle_callback(self, msg_in: Int8):
        
        if self.state == "PAUSE": #current DRIVE next PAUSE
            if msg_in.data == 1:
                self.state = "SMALL_OBSTACLE_DETECTED"
                self.timer.cancel()
                self.fsm_step()
            elif msg_in.data == 2:
                self.state = "SMALL_OBSTACLE_DETECTED"
                self.timer.cancel()
                self.fsm_step()
            else:
                pass

    def fsm_step(self):
        
        if self.state == "SCAN_TURN_RESOLVE":
            
            if self.scan_counter == 0:
                self.distance_array1 = []
                self.scan_counter += 1
                self.state = "SCAN_STOP"
                
                msg = Twist()
                self.publisher.publish(msg)
                self.timer.cancel()
                self.timer = self.create_timer(0, self.fsm_step)    

            elif self.scan_counter > 0 and self.scan_counter < 6:
                self.distance_array1.append(self.distance)
                
                self.state = "SCAN_STOP"
                self.scan_counter += 1
                
                msg = Twist()
                msg.angular.z = -3.14
                self.publisher.publish(msg)
                self.timer.cancel()
                self.timer = self.create_timer(0.1, self.fsm_step)

            elif self.scan_counter == 6:
                self.distance_array1.append(self.distance)
                self.scan_counter = 0

                self.timer.cancel()
                self.timer = self.create_timer(0, self.fsm_step)    
                self.state = "RESOLVE_SCAN_RESULT"

            else:
                pass #err
            
        elif self.state == "SCAN_STOP":
            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(0.15, self.fsm_step)    
            self.state = "SCAN_TURN_RESOLVE"
        
        elif self.state == "SMALL_OBSTACLE_DETECTED":
            self.get_logger().info("SMALL_OBSTACLE_DETECTED")
            msg = Twist()
            msg.angular.z = 3.14
            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(0.25, self.fsm_step)    
            self.state = "SCAN_TURN_RESOLVE"
            
        elif self.state == "SCAN_FOR_OBSTACLES":
            self.get_logger().info("SCAN_FOR_OBSTACLES")
            self.start_index = self.distance_index
            msg = Twist()
            msg.angular.z = -2.355
            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(0.6693, self.fsm_step)    
            self.state = "RESOLVE_SCAN_RESULT"

        elif self.state == "RESOLVE_SCAN_RESULT":
            self.get_logger().info("RESOLVE_SCAN_RESULT")
            msg = Twist()
            self.publisher.publish(msg)
            self.timer.cancel()

            for item in self.distance_array1:
                print(item)

            left = min(self.distance_array1[0], min(self.distance_array1[1], self.distance_array1[2]))
            right = min(self.distance_array1[3], min(self.distance_array1[4], self.distance_array1[5]))

            print(f"left: {left}")
            print(f"right: {right}")

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

            # for i in range(0, 5):
            #     self.get_logger().info(f"distance index {i} value {self.distance_array[(self.start_index + i) % 5]}")
            #     if self.distance_array[(self.start_index + i) % 5] < 0.4:
                    
            #         if i < 2:
            #             self.state = "RECENTER_LEFT"

            #         elif i > 2:
            #             self.state = "EVADE_LEFT"

            #         else:
            #             self.state = "OBSTACLE"

            #         self.timer = self.create_timer(0, self.fsm_step)
            #         return

            # msg = Twist()
            # msg.angular.z = 3.14
            # self.publisher.publish(msg)
            # time.sleep(0.25)
            # msg.angular.z = 0.0
            # self.publisher.publish(msg)

            # self.state = "DRIVE"
            # self.timer = self.create_timer(0, self.fsm_step)


        #     msg = Twist()
        #     self.publisher.publish(msg)

        #     msg.angular.z = 3.14
        #     self.publisher.publish(msg)
        #     time.sleep(0.25)
        #     msg.angular.z = 0.0
        #     self.publisher.publish(msg)
        #     self.timer = self.create_timer(0.15, self.fsm_step)
        #     self.state = "CHECK_LIGHT_LEFT"

        # elif self.state == "CHECK_LIGHT_LEFT":
        #     self.timer.cancel()

        #     self.get_logger().info("check left")
        #     self.get_logger().info(str(self.distance))
        #     lelf = self.distance
        #     if self.distance < 0.4:
        #         self.get_logger().info("found left")
        #         self.state = "EVADE_RIGHT"
        #         msg = Twist()
        #         msg.angular.z = 3.14
        #         self.publisher.publish(msg)
        #         time.sleep(0.25)
        #         self.fsm_step()
        #         return

        #     msg = Twist()
        #     msg.angular.z = -3.14
        #     self.publisher.publish(msg)
        #     time.sleep(0.5)
        #     msg.angular.z = 0.0
        #     self.publisher.publish(msg)
            
        #     self.timer = self.create_timer(0.15, self.fsm_step)
        #     self.state = "CHECK_LIGHT_RIGHT"

        # elif self.state == "CHECK_LIGHT_RIGHT":
        #     self.timer.cancel()
        #     self.get_logger().info("check right")
        #     self.get_logger().info(str(self.distance))
        #     if self.distance < 0.4:
        #         self.get_logger().info("found right")
        #         self.state = "EVADE_LEFT"
        #         msg = Twist()
        #         msg.angular.z = 3.14
        #         self.publisher.publish(msg)
        #         time.sleep(0.25)
        #         self.fsm_step()
        #         return

        #     self.get_logger().info("no obstacle found")
        #     msg = Twist()
        #     msg.angular.z = 3.14
        #     self.publisher.publish(msg)
        #     time.sleep(0.25)
        #     self.timer = self.create_timer(0.5, self.fsm_step)
        #     self.state = "DRIVE"

        elif self.state == "EVADE_LEFT":
            self.get_logger().info("EVADE_LEFT")
            msg_out = Twist()
            msg_out.linear.x = 0.0
            msg_out.angular.z = 3.14
            self.publisher.publish(msg_out)
            self.timer.cancel()
            self.timer = self.create_timer(0.5, self.fsm_step)
            self.state = "RECENTER_RIGHT"

        elif self.state == "RECENTER_RIGHT":
            self.get_logger().info("RECENTER_RIGHT")
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

        elif self.state == "EVADE_RIGHT":
            self.get_logger().info("EVADE_RIGHT")
            msg_out = Twist()
            msg_out.linear.x = 0.0
            msg_out.angular.z = -3.14
            self.publisher.publish(msg_out)
            self.timer.cancel()
            self.timer = self.create_timer(0.25, self.fsm_step)
            self.state = "RECENTER_LEFT"

        elif self.state == "RECENTER_LEFT":
            self.get_logger().info("RECENTER_LEFT")
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

        elif self.state == "START":
            self.get_logger().info("START")
            if random.random() < 0.5:
                self.state = "DRIVE"
            else:
                 self.state = "DRIVE"
            self.fsm_step()

        elif self.state == "PAUSE":
            self.get_logger().info("PAUSE")
            self.state = "START"
            
            msg = Twist()
            self.publisher.publish(msg)
            
            self.timer.cancel()
            self.timer = self.create_timer(1, self.fsm_step)

        elif self.state == "TURN":
            self.get_logger().info("TURN")
            self.state = "DRIVE"
            speed = random.uniform(-0.785, 0.785)
            self.get_logger().info(str(speed))
            msg = Twist()
            msg.angular.z = speed
            self.publisher.publish(msg)

            self.timer.cancel()
            duration = random.uniform(0, 2)
            self.get_logger().info(str(duration))
            self.timer = self.create_timer(duration, self.fsm_step)

        elif self.state == "DRIVE":
            self.get_logger().info("DRIVE")
            self.state = "PAUSE"

            speed = random.uniform(0, 0.30695)
            speed = 0.15
            msg = Twist()
            msg.linear.x = speed
            self.get_logger().info(str(speed))
            self.publisher.publish(msg)

            self.timer.cancel()
            duration = random.uniform(0, 5)
            duration = 2
            self.get_logger().info(str(duration))
            self.timer = self.create_timer(duration, self.fsm_step)

        elif self.state == "OBSTACLE":
            self.state = "FIRST_SIDE_OBSTACLE"
            self.get_logger().info("OBSTACLE")
            self.timer.cancel()
            
            #reverse
            msg = Twist()
            msg.linear.x = -0.2
            self.publisher.publish(msg)
            time.sleep(0.5)

            #turn in random direction
            msg.linear.x = 0.0
            self.chance = random.random()
            self.get_logger().info(str(self.chance))
            
            if self.chance < 0.5:
                msg.angular.z = 3.14
            else:
                msg.angular.z = -3.14
            
            self.get_logger().info("turning")

            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(0.5, self.fsm_step)
        
        elif self.state == "FIRST_SIDE_OBSTACLE":
            self.get_logger().info("FIRST_SIDE_OBSTACLE")
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
            self.get_logger().info("SECOND_SIDE_OBSTACLE")
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

    def scan(self):
        self.scan_results = []

        msg = Twist()
        msg.angular.z = 3.14
        self.publisher.publish(msg)
        self.timer.cancel()
        self.timer = self.create_timer(0.25, next(self.scan_gen))
        yield

        msg = Twist()
        msg.angular.z = -3.14
        self.publisher.publish(msg)
        self.timer.cancel()
        self.timer = self.create_timer(0.083, next(self.scan_gen))
        self.get_logger().info("scan distances:")
        for i in range(0, 5):
            self.get_logger().info(str(self.distance))
            self.scan_results.append(self.distance)
            yield

        msg = Twist()
        self.publisher.publish(msg)
        self.timer.cancel()

        for i in range(0, 5):
            if self.scan_results[i] < 0.4:
                if i < 2:
                    self.state = "RECENTER_RIGHT"
                elif i > 2:
                    self.state = "EVADE_RIGHT"
                else:
                    self.state = "OBSTACLE"
                
                self.timer = self.create_timer(0, self.fsm_step)
                yield




def main():
    rclpy.init()
    node = WanderingNode()
    rclpy.spin(node)
    rclpy.shutdown()