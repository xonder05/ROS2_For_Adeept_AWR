import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from mpu6050 import mpu6050

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('scanning_period', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.scanning_period = self.get_parameter('scanning_period').get_parameter_value().double_value
        
        self.subscriber = self.create_subscription(Twist, "/drive_directions", self.twist_callback, 10)
        self.publisher = self.create_publisher(Twist, "/imu_node/sensor_reading", 10)
        self.colision_publisher = self.create_publisher(Bool, "/imu_node/colision_warning", 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        self.sensor = sensor = mpu6050(0x68)

        self.linear_command = 0
        self.angular_command = 0
        self.history = [0.0] * 5

        self.get_logger().info("InitDone")

    def twist_callback(self, msg: Twist):
        self.linear_command = msg.linear.x
        self.angular_command = msg.angular.z

    def timer_callback(self):
        msg = Twist()
        
        x = 0
        for i in range(0,10):
            x = x + self.sensor.get_accel_data()['x']
        accel_data = x / 10.0

        msg.linear.x = accel_data
        msg.angular.z = self.sensor.get_gyro_data()['y']
        self.publisher.publish(msg)

        msg = Bool()

        maximum = 0
        for item in self.history:
            maximum = max(abs(item), maximum)

        if maximum < 0.5 and self.linear_command == 0:
            msg.data = False #calm

        elif maximum < 0.5 and self.linear_command > 0:
            msg.data = True #stopped wheels not spining

        # elif maximum < 1.75 and self.linear_command > 0:
        #     msg.data = True #stopped wheels spining

        else:
            self.data = False #driving

        self.colision_publisher.publish(msg)

        self.history.append(accel_data)
        self.history.pop(0)

def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()
