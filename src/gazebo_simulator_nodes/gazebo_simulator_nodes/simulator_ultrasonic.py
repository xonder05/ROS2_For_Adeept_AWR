import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Bool

class SimulatorUltrasonic(Node):

    def __init__(self):
        super().__init__("simulator_ultrasonic")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic_name', rclpy.Parameter.Type.STRING),
                ('obstacle_warning_distance', rclpy.Parameter.Type.DOUBLE),
                ('side_obstacle_minimum_detection_distance', rclpy.Parameter.Type.DOUBLE),
                ('side_obstacle_maximum_detection_distance', rclpy.Parameter.Type.DOUBLE)
            ]
        )
        self.gazebo_topic_name = self.get_parameter('gazebo_topic_name').get_parameter_value().string_value
        self.obstacle_warning_distance = self.get_parameter('obstacle_warning_distance').get_parameter_value().double_value
        self.side_obstacle_minimum_detection_distance = self.get_parameter('side_obstacle_minimum_detection_distance').get_parameter_value().double_value
        self.side_obstacle_maximum_detection_distance = self.get_parameter('side_obstacle_maximum_detection_distance').get_parameter_value().double_value

        self.subscriber = self.create_subscription(LaserScan, self.gazebo_topic_name, self.callback, 10)
        self.distance_publisher = self.create_publisher(Float32, "/ultrasonic_distance", 10)
        self.obstacle_warning_publisher = self.create_publisher(Bool, "/ultrasonic_obstacle_warning", 10)
        self.obstacle_disappearance_warning_publisher = self.create_publisher(Bool, "/ultrasonic_obstacle_disappearance_warning", 10)

        self.previous_distance = 0

        self.get_logger().info("InitDone")

    def callback(self, msg: LaserScan):
        
        #interpretation of lidar data to simulate ultrasonic behaviour
        left = msg.ranges[0]
        middle = msg.ranges[1]
        right = msg.ranges[2]
        distance = min(left, min(middle, right))

        #seding the percieved distance
        msg = Float32()
        msg.data = distance
        self.distance_publisher.publish(msg)

        #sending warning about close obstacle
        msg = Bool()
        if distance < self.obstacle_warning_distance:
            msg.data = True
        else:
            msg.data = False
        self.obstacle_warning_publisher.publish(msg)

        #sending warning about obstacle that is no longer visible but is on trajectory that could colide with the robot
        msg = Bool()
        if distance - self.previous_distance > 0.2 and self.previous_distance > self.side_obstacle_minimum_detection_distance and self.previous_distance < self.side_obstacle_maximum_detection_distance:
            msg.data = True
        else:
            msg.data = False
        self.obstacle_disappearance_warning_publisher.publish(msg)

        self.previous_distance = distance

def main():
    rclpy.init()
    node = SimulatorUltrasonic()
    rclpy.spin(node)
    rclpy.shutdown()