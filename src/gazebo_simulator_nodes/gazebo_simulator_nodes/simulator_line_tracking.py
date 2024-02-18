import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from adeept_awr_interfaces.msg import LineTracking

class SimulatorLineTracking(Node):
    def __init__(self):
        super().__init__("simulator_line_tracking")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic', rclpy.Parameter.Type.STRING),
                ('ros_topic', rclpy.Parameter.Type.STRING)
            ]
        )
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(Image, self.gazebo_topic, self.callback, 10)
        self.publisher = self.create_publisher(LineTracking, self.ros_topic , 10)

        self.buffer = {}
        self.get_logger().info("InitDone")

    def callback(self, msg: Image):

        #first message for time
        if (str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)) not in self.buffer:
            self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)] = {}
            self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)][msg.header.frame_id] = msg.data[0] * 0.299 + msg.data[0] * 0.587 + msg.data[0] * 0.114

        #subsequent messages for time
        else:
            self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)][msg.header.frame_id] = msg.data[0] * 0.299 + msg.data[0] * 0.587 + msg.data[0] * 0.114

            #last messages for time
            if len(self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)]) == 3:
                msg_out = LineTracking()
                msg_out.left = self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)]["adeept_awr/left/line_tracking/camera"] < 50
                msg_out.middle = self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)]["adeept_awr/middle/line_tracking/camera"] < 50
                msg_out.right = self.buffer[str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)]["adeept_awr/right/line_tracking/camera"] < 50
                self.publisher.publish(msg_out)

def main():
    rclpy.init()
    node = SimulatorLineTracking()
    rclpy.spin(node)
    rclpy.shutdown()
