import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
import tf2_ros

import math
import transforms3d

from mpu6050 import mpu6050

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        
        self.init_params()

        self.subscriber = self.create_subscription(Twist, "/cmd_vel", self.twist_callback, 10)
        self.publisher = self.create_publisher(Twist, "/imu_data", 10)
        self.colision_publisher = self.create_publisher(Bool, "/not_moving_warning", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.05, self.publish_tf_data)
        
        self.sensor = sensor = mpu6050(0x68)
        sensor.set_filter_range(0x05)

        self.linear_command = 0
        self.angular_command = 0
        self.prev_time = self.get_clock().now()
        self.prev_accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.prev_gyro_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.absolute_position = {'x': 0.0, 'y': 0.0}
        self.inertia = 10
        self.warning_count = 0

        self.get_logger().info("InitDone")

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('acc_x_offset', rclpy.Parameter.Type.DOUBLE),
                ('acc_y_offset', rclpy.Parameter.Type.DOUBLE),
                ('acc_z_offset', rclpy.Parameter.Type.DOUBLE),
                ('gyro_x_offset', rclpy.Parameter.Type.DOUBLE),
                ('gyro_y_offset', rclpy.Parameter.Type.DOUBLE),
                ('gyro_z_offset', rclpy.Parameter.Type.DOUBLE),
                ('acc_x_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('acc_y_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('acc_z_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('gyro_x_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('gyro_y_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('gyro_z_low_cutoff', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.acc_x_offset = self.get_parameter('acc_x_offset').get_parameter_value().double_value
        self.acc_y_offset = self.get_parameter('acc_y_offset').get_parameter_value().double_value
        self.acc_z_offset = self.get_parameter('acc_z_offset').get_parameter_value().double_value
        self.gyro_x_offset = self.get_parameter('gyro_x_offset').get_parameter_value().double_value
        self.gyro_y_offset = self.get_parameter('gyro_y_offset').get_parameter_value().double_value
        self.gyro_z_offset = self.get_parameter('gyro_z_offset').get_parameter_value().double_value
        self.acc_x_low_cutoff = self.get_parameter('acc_x_low_cutoff').get_parameter_value().double_value
        self.acc_y_low_cutoff = self.get_parameter('acc_y_low_cutoff').get_parameter_value().double_value
        self.acc_z_low_cutoff = self.get_parameter('acc_z_low_cutoff').get_parameter_value().double_value
        self.gyro_x_low_cutoff = self.get_parameter('gyro_x_low_cutoff').get_parameter_value().double_value
        self.gyro_y_low_cutoff = self.get_parameter('gyro_y_low_cutoff').get_parameter_value().double_value
        self.gyro_z_low_cutoff = self.get_parameter('gyro_z_low_cutoff').get_parameter_value().double_value
    
    def twist_callback(self, msg: Twist):
        self.linear_command = msg.linear.x
        self.angular_command = msg.angular.z

    def get_accel_data(self):
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        #average value from x samples (not used in final solution but the option is there)
        for i in range(0, 1):
            new_data = self.sensor.get_accel_data()

            accel_data['x'] += (new_data['x'] + self.acc_x_offset)
            accel_data['y'] += (new_data['y'] + self.acc_y_offset)
            accel_data['z'] += (new_data['z'] + self.acc_z_offset)

        accel_data['x'] = accel_data['x'] / 1.0
        accel_data['y'] = accel_data['y'] / 1.0
        accel_data['z'] = accel_data['z'] / 1.0

        #cut low values (so the robot does not drift while stationary)
        if abs(accel_data['x']) < self.acc_x_low_cutoff:
            accel_data['x'] = 0

        if abs(accel_data['y']) < self.acc_y_low_cutoff:
            accel_data['y'] = 0

        if abs(accel_data['z']) < self.acc_z_low_cutoff:
            accel_data['z'] = 0

        #align physical axis with those used by ros2
        accel_data['x'] *= -1

        tmp = accel_data['y']
        accel_data['y'] = accel_data['z']
        accel_data['z'] = tmp

        return accel_data

    def get_gyro_data(self):
        gyro_data_deg = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        #average value from x samples (not used in final solution but the option is there)
        for i in range(0, 1):
            new_data = self.sensor.get_gyro_data()

            gyro_data_deg['x'] += (new_data['x'] + self.gyro_x_offset)
            gyro_data_deg['y'] += (new_data['y'] + self.gyro_y_offset)
            gyro_data_deg['z'] += (new_data['z'] + self.gyro_z_offset)

        gyro_data_deg['x'] = gyro_data_deg['x'] / 1.0
        gyro_data_deg['y'] = gyro_data_deg['y'] / 1.0
        gyro_data_deg['z'] = gyro_data_deg['z'] / 1.0

        #cut low values (so the robot does not drift while stationary)
        if abs(gyro_data_deg['x']) < self.gyro_x_low_cutoff:
            gyro_data_deg['x'] = 0

        if abs(gyro_data_deg['y']) < self.gyro_y_low_cutoff:
            gyro_data_deg['y'] = 0
        
        if abs(gyro_data_deg['z']) < self.gyro_z_low_cutoff:
            gyro_data_deg['z'] = 0

        #align physical axis with those used by ros2
        tmp = gyro_data_deg['y']
        gyro_data_deg['y'] = gyro_data_deg['z']
        gyro_data_deg['z'] = tmp

        gyro_data_deg['z'] *= -1

        #convertion from degrees to radians
        gyro_data = {
            'x': math.radians(gyro_data_deg['x']),
            'y': math.radians(gyro_data_deg['y']),
            'z': math.radians(gyro_data_deg['z'])
        }

        return gyro_data

    def publish_tf_data(self):
        #get information for calcualtions
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        gyro_data = self.get_gyro_data()
        
        self.orientation['yaw'] += (gyro_data['z'] + self.prev_gyro_data['z']) / 2 * dt

        accel_data = self.get_accel_data()

        #don't move robot when it should be stationary
        if self.linear_command == 0:
            #to let the robot stop when the command becomes zero
            if self.inertia < 10:
                self.inertia += 1
            else:
                self.velocity['x'] = 0
        else:
            self.velocity['x'] += (accel_data['x'] + self.prev_accel_data['x']) / 2 * dt
            self.inertia = 0

        delta_x = self.velocity['x'] * math.cos(self.orientation['yaw']) * dt
        delta_y = self.velocity['x'] * math.sin(self.orientation['yaw']) * dt

        self.absolute_position['x'] += delta_x
        self.absolute_position['y'] += delta_y

        #publish imu data
        msg = Twist()
        msg.linear.x = float(self.velocity['x'])
        msg.angular.z = float(self.orientation['yaw'])
        self.publisher.publish(msg)

        #publish not moving warning
        if self.linear_command > 0 and abs(self.linear_command - self.velocity['x']) > 0.25:
            if self.warning_count > 5:
                msg = Bool()
                msg.data = True
                self.colision_publisher.publish(msg)
            else:
                self.warning_count += 1
        else:
            self.warning_count = 0

            msg = Bool()
            msg.data = False
            self.colision_publisher.publish(msg)
        
        #publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.absolute_position['x']
        transform.transform.translation.y = self.absolute_position['y']
        transform.transform.translation.z = 0.0
        quat = transforms3d.euler.euler2quat(0, 0, self.orientation['yaw'])
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        self.tf_broadcaster.sendTransform(transform)

        #prepare for next iteration
        self.prev_time = current_time
        self.prev_accel_data = accel_data
        self.prev_gyro_data = gyro_data

def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()
