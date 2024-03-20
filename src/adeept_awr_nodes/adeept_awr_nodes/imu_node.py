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
        
        self.subscriber = self.create_subscription(Twist, "/cmd_vel", self.twist_callback, 10)
        self.publisher = self.create_publisher(Twist, "/imu_node/sensor_reading", 10)
        self.colision_publisher = self.create_publisher(Bool, "/imu_node/colision_warning", 10)
        self.tf_timer = self.create_timer(0.01, self.publish_tf_data)
        
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
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        self.absolute_position = {'x': 0.0, 'y': 0.0}

        self.max = 0

        self.get_logger().info("InitDone")

    def twist_callback(self, msg: Twist):
        self.linear_command = msg.linear.x
        self.angular_command = msg.angular.z


    def get_accel_data(self):
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        #average value from x samples
        for i in range(0, 1):
            new_data = self.sensor.get_accel_data()

            accel_data['x'] += (new_data['x'] - 0.165931074)
            accel_data['y'] += (new_data['y'] + 10.001548310205276)
            accel_data['z'] += (new_data['z'] - 0.141471371)

        accel_data['x'] = accel_data['x'] / 1.0
        accel_data['y'] = accel_data['y'] / 1.0
        accel_data['z'] = accel_data['z'] / 1.0

        #cut low values (so the robot does not drift while stationary)
        if abs(accel_data['x']) < 0.15:
            accel_data['x'] = 0

        if abs(accel_data['y']) < 0.05:
            accel_data['y'] = 0
        
        if abs(accel_data['z']) < 0.5:
            accel_data['z'] = 0

        #align physical axis with those used by ros2
        accel_data['x'] *= -1

        tmp = accel_data['y']
        accel_data['y'] = accel_data['z']
        accel_data['z'] = tmp

        return accel_data

    def get_gyro_data(self):
        gyro_data_deg = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        #average value from x samples
        for i in range(0, 1):
            new_data = self.sensor.get_gyro_data()

            gyro_data_deg['x'] += (new_data['x'] - -1.7608719847328245)
            gyro_data_deg['y'] += (new_data['y'] - -0.4370078625954107)
            gyro_data_deg['z'] += (new_data['z'] + 0.10326725190839636)

        gyro_data_deg['x'] = gyro_data_deg['x'] / 1.0
        gyro_data_deg['y'] = gyro_data_deg['y'] / 1.0
        gyro_data_deg['z'] = gyro_data_deg['z'] / 1.0

        #cut low values (so the robot does not drift while stationary)
        if abs(gyro_data_deg['x']) < 5:
            gyro_data_deg['x'] = 0

        if abs(gyro_data_deg['y']) < 1:
            gyro_data_deg['y'] = 0
        
        if abs(gyro_data_deg['z']) < 5:
            gyro_data_deg['z'] = 0

        #align physical axis with those used by ros2
        tmp = gyro_data_deg['y']
        gyro_data_deg['y'] = gyro_data_deg['z']
        gyro_data_deg['z'] = tmp

        #convertion from degrees to radians
        gyro_data = {
            'x': math.radians(gyro_data_deg['x']),
            'y': math.radians(gyro_data_deg['y']),
            'z': math.radians(gyro_data_deg['z'])
        }

        return gyro_data

    def publish_tf_data(self):
        
        #get data
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
    
        accel_data = self.get_accel_data()

        gyro_data = self.get_gyro_data()

        #calculate velocities, rotations and positions
        self.prev_velocity['x'] += (accel_data['x'] + self.prev_accel_data['x']) / 2 * dt

        delta_x = self.prev_velocity['x'] * math.cos(self.orientation['yaw']) * dt
        delta_y = self.prev_velocity['x'] * math.sin(self.orientation['yaw']) * dt

        self.absolute_position['x'] += delta_x
        self.absolute_position['y'] += delta_y

        self.orientation['yaw'] += gyro_data['z'] * dt

        #publish imu data
        msg = Twist()
        msg.linear.x = self.prev_velocity['x']
        msg.angular.z = self.orientation['yaw']
        self.publisher.publish(msg)

        #publish not moving warning
        if accel_data['x'] < 0.1 and (self.linear_command - self.prev_velocity['x']) > 0.2:
            if self.strike:
                msg = Bool()
                msg.data = True
                self.publisher.publish(msg)
            else:
                self.strike = True
        else:
            self.strike = False

            msg = Bool()
            msg.data = False
            self.publisher.publish(msg)
        
        #publish transform
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

        if abs(self.prev_velocity['x']) < 0.01:
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

        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()
