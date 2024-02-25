import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from interfaces.msg import LineTracking

class SimulatorLineTracking(Node):
    def __init__(self):
        super().__init__("simulator_line_tracking")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic', rclpy.Parameter.Type.STRING),
                ('ros_topic', rclpy.Parameter.Type.STRING),
                ('left_sensor_name', rclpy.Parameter.Type.STRING),
                ('middle_sensor_name', rclpy.Parameter.Type.STRING),
                ('right_sensor_name', rclpy.Parameter.Type.STRING),
                ('line_detection_threshold', rclpy.Parameter.Type.INTEGER)
            ]
        )
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.left_sensor_name = self.get_parameter('left_sensor_name').get_parameter_value().string_value
        self.middle_sensor_name = self.get_parameter('middle_sensor_name').get_parameter_value().string_value
        self.right_sensor_name = self.get_parameter('right_sensor_name').get_parameter_value().string_value
        self.line_detection_threshold = self.get_parameter('line_detection_threshold').get_parameter_value().integer_value

        self.subscription = self.create_subscription(Image, self.gazebo_topic, self.callback, 10)
        self.publisher = self.create_publisher(LineTracking, self.ros_topic , 10)

        self.buffer = {} #sensor data arrive in multiple messages so buffer is used to store them until all arrive

        self.get_logger().info("InitDone")

    def callback(self, msg: Image):
        
        #convert rgb value of recieved pixel to grayscale
        grayscale = msg.data[0] * 0.299 + msg.data[1] * 0.587 + msg.data[2] * 0.114

        #first message in block
        if (str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)) not in self.buffer:
            self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)] = {}
            self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)][msg.header.frame_id] = grayscale

        #subsequent messages
        else:
            self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)][msg.header.frame_id] = grayscale

            #if all messages for specific block arrived
            if len(self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)]) == 3:
                msg_out = LineTracking()
                msg_out.left = self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)][self.left_sensor_name] < self.line_detection_threshold
                msg_out.middle = self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)][self.middle_sensor_name] < self.line_detection_threshold
                msg_out.right = self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)][self.right_sensor_name] < self.line_detection_threshold
                self.publisher.publish(msg_out)
                del self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)]

def main():
    rclpy.init()
    node = SimulatorLineTracking()
    rclpy.spin(node)
    rclpy.shutdown()
