import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
from interfaces.action import MCC

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__("motor_controller_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('base_turning_speed', rclpy.Parameter.Type.DOUBLE),
                ('imu_topic', rclpy.Parameter.Type.STRING),
                ('commands_topic', rclpy.Parameter.Type.STRING),
            ]
        )
        self.base_turning_speed = self.get_parameter('base_turning_speed').get_parameter_value().double_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.commands_topic = self.get_parameter('commands_topic').get_parameter_value().string_value

        self.subscriber = self.create_subscription(Twist, self.imu_topic, self.imu_callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.action_server = ActionServer(self, MCC, self.commands_topic,
                                            goal_callback=self.goal_callback,
                                            handle_accepted_callback=self.handle_accepted_callback,
                                            execute_callback=self.execute_callback,
                                            cancel_callback=self.cancel_callback)

        self.current_angle = 0
        self.goal_handle = None
        self.canceled_goal_handle = None
        self.waited = False

        self.get_logger().info("InitDone")

    #getting imu readings
    def imu_callback(self, msg):
        self.current_angle = msg.angular.z

        #cancel
        if self.canceled_goal_handle is not None and self.canceled_goal_handle.is_cancel_requested:
            self.canceled_goal_handle.execute()
            return

        #active handle
        if self.goal_handle is not None and self.goal_handle.is_active:
            distance = (self.current_angle - self.goal_angle)
            
            #changing speed based on remainig distance
            if abs(distance) < 0.02:
                if self.waited:
                    self.waited = False
                    self.goal_handle.execute()
                    return
                else:
                    self.waited = True
                    speed = 0.0
            elif abs(distance) < 0.3:
                speed = 4.71
            elif abs(distance < 1):
                speed = 4.71
            else:
                speed = 6.24
    
            #simple publish
            msg = Twist()
            if distance > 0:
                msg.angular.z = -speed
            else:
                msg.angular.z = speed
            self.publisher.publish(msg)

    #only one action at a time, two nodes should not control the motor simultaneously
    def goal_callback(self, goal_request):
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().info(f"rejecting goal {goal_request.angle}")
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.canceled_goal_handle = goal_handle
        self.goal_handle = None
        return CancelResponse.ACCEPT

    #save goal angle and handle
    def handle_accepted_callback(self, goal_handle):
        self.goal_angle = self.current_angle + (goal_handle.request.angle)
        self.goal_handle = goal_handle
    
    #target angle reached
    def execute_callback(self, goal_handle):
        msg = Twist()
        self.publisher.publish(msg)
        
        if goal_handle.is_cancel_requested:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        
        result = MCC.Result()
        return result

def main():
    rclpy.init()
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()