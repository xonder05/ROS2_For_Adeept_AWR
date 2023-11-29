import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adeept_awr_output_devices',
            executable='dc_motor_node',
            namespace="",
        ),
        Node(
            package='adeept_awr_output_devices',
            executable='servo_node',
            namespace="",
        ),
        Node(
            package='adeept_awr_output_devices',
            executable='servo_node',
            namespace="",
        ),
        Node(
            package='adeept_awr_input_sensors',
            executable='pi_camera_node',
            namespace="",
        ),
        Node(
            package='adeept_awr_web_controller',
            executable='web_controller_node',
            namespace="",
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rosbridge_server'),
                    '/opt/ros/iron/share/rosbridge_server/launch/rosbridge_websocket_launch.xml'))
        ),
    ])