import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import random
import time

class WanderingNode(Node):

    def __init__(self):
        super().__init__("wandering_node")
        self.obstacle_subscriber = self.create_subscription(Bool, "/obstacle_detected", self.obstacle_callback, 10)
        self.publisher = self.create_publisher(Twist, "/drive_directions", 10)
        self.state = "START"
        self.timer = self.create_timer(1, self.fsm_step)

        self.get_logger().info("InitDone")
        self.fsm_step()

    def obstacle_callback(self, msg: Bool):
        self.obstacle = msg.data
        if msg.data and self.state not in ["OBSTACLE", "FIRST_SIDE_OBSTACLE", "SECOND_SIDE_OBSTACLE"]:
            self.state = "OBSTACLE"

    def fsm_step(self):
        
        if self.state == "START":
            self.get_logger().info("START")
            if random.random() < 0.5:
                self.state = "TURN"
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

            speed = random.uniform(0, 0.153475)
            msg = Twist()
            msg.linear.x = speed
            self.get_logger().info(str(speed))
            self.publisher.publish(msg)

            self.timer.cancel()
            duration = random.uniform(0, 5)
            self.get_logger().info(str(duration))
            self.timer = self.create_timer(duration, self.fsm_step)

        elif self.state == "OBSTACLE":
            self.state = "FIRST_SIDE_OBSTACLE"
            self.get_logger().info("OBSTACLE")
            self.timer.cancel()
            
            #reverse
            msg = Twist()
            msg.linear.x = -0.0767375
            self.publisher.publish(msg)
            time.sleep(0.5)

            #turn in random direction
            msg = Twist()
            self.chance = random.random()
            self.get_logger().info(str(self.chance))
            
            if self.chance < 0.5:
                msg.angular.z = 0.785
            else:
                msg.angular.z = -0.785
            
            self.publisher.publish(msg)
            self.timer.cancel()
            self.timer = self.create_timer(2, self.fsm_step)
        
        elif self.state == "FIRST_SIDE_OBSTACLE":
            self.get_logger().info("FIRST_SIDE_OBSTACLE")
            if self.obstacle:
                self.state = "SECOND_SIDE_OBSTACLE"
                msg = Twist()
                msg.angular.z = 1.57
                self.publisher.publish(msg)
                
                self.timer.cancel()
                self.timer = self.create_timer(2, self.fsm_step)
            else:
                self.state = "DRIVE"
                self.fsm_step()

        elif self.state == "SECOND_SIDE_OBSTACLE":
            self.get_logger().info("SECOND_SIDE_OBSTACLE")
            self.state = "DRIVE"
            if self.obstacle:
                msg = Twist()
                if self.chance < 0.5:
                    msg.angular.z = -0.785
                else:
                    msg.angular.z = 0.785
                
                self.publisher.publish(msg)
                
                self.timer.cancel()
                self.timer = self.create_timer(2, self.fsm_step)
            else:
                self.fsm_step()

def main():
    rclpy.init()
    node = WanderingNode()
    rclpy.spin(node)
    rclpy.shutdown()