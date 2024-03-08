import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math

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
        self.timer = self.create_timer(0.005, self.publish_tf_data)
        
        self.sensor = sensor = mpu6050(0x68)
        sensor.set_filter_range(0x05)

        self.linear_command = 0
        self.angular_command = 0
        self.history = [0.0] * 5

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize state variables
        self.prev_time = self.get_clock().now()
        self.prev_accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.prev_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.prev_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        self.absolute_position = {'x': 0.0, 'y': 0.0}

        self.get_logger().info("InitDone")

    def twist_callback(self, msg: Twist):
        self.linear_command = msg.linear.x
        self.angular_command = msg.angular.z

    def publish_tf_data(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        for i in range(0, 10):
        
            new_data = self.sensor.get_accel_data()

            accel_data['x'] += (new_data['x'] - 0.165931074)
            accel_data['y'] += (new_data['y'] + 10.001548310205276)
            accel_data['z'] += (new_data['z'] - 0.141471371)

        accel_data['x'] = accel_data['x'] / 10.0
        accel_data['y'] = accel_data['y'] / 10.0
        accel_data['z'] = accel_data['z'] / 10.0

        accel_data['x'] *= -1

        if abs(accel_data['x']) < 0.1:
            accel_data['x'] = 0

        if abs(accel_data['y']) < 0.05:
            accel_data['y'] = 0
        
        if abs(accel_data['z']) < 0.5:
            accel_data['z'] = 0

        tmp = accel_data['y']
        accel_data['y'] = accel_data['z']
        accel_data['z'] = tmp


        gyro_data_deg = self.sensor.get_gyro_data()

        gyro_data_deg['x'] -= -1.7608719847328245
        gyro_data_deg['y'] -= -0.4370078625954107
        gyro_data_deg['z'] += 0.10326725190839636

        if abs(gyro_data_deg['x']) < 5:
            gyro_data_deg['x'] = 0

        if abs(gyro_data_deg['y']) < 1:
            gyro_data_deg['y'] = 0
        
        if abs(gyro_data_deg['z']) < 5:
            gyro_data_deg['z'] = 0

        tmp = gyro_data_deg['y']
        gyro_data_deg['y'] = gyro_data_deg['z']
        gyro_data_deg['z'] = tmp

        gyro_data = {
            'x': math.radians(gyro_data_deg['x']),
            'y': math.radians(gyro_data_deg['y']),
            'z': math.radians(gyro_data_deg['z'])
        }

        for axis in ['x', 'y', 'z']:
            self.prev_velocity[axis] += (accel_data[axis] + self.prev_accel_data[axis]) / 2 * dt
            self.prev_position[axis] += self.prev_velocity[axis]

        # Calculate change in position based on current linear velocity and orientation
        delta_x = self.prev_velocity['x'] * math.cos(self.orientation['yaw']) * dt
        delta_y = self.prev_velocity['x'] * math.sin(self.orientation['yaw']) * dt

        # Update absolute x and y positions
        self.absolute_position['x'] += delta_x
        self.absolute_position['y'] += delta_y

        # if (abs(accel_data['x']) > 0 or abs(self.prev_accel_data['x']) > 0):
            # self.get_logger().info(f"{self.prev_velocity['x']} = {(accel_data['x'] + self.prev_accel_data['x']) / 2 * dt}")

        self.orientation['yaw'] += gyro_data['z'] * dt
        
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.absolute_position['x']
        transform.transform.translation.y = self.absolute_position['y']
        transform.transform.translation.z = 0.0
        transform.transform.rotation = self.euler_to_quaternion(0, 0, self.orientation['yaw'])
        self.tf_broadcaster.sendTransform(transform)

        self.prev_time = current_time
        self.prev_accel_data = accel_data

        if abs(self.prev_velocity['x']) < 0.1:
            self.prev_velocity['x'] = 0

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        # self.get_logger().info(f"{sy}*{cp}*{cr}-{cy}*{sp}*{sr}")
        # self.get_logger().info(f"x: {qx}, y: {qy}, z:{qz}, w: {qw}")

        return Quaternion(x=qx, y=qy, z=qz, w=qw)


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
