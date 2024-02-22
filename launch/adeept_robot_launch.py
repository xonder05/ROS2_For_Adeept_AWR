import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/dc_motor_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/servo_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/pi_camera_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/rgb_led_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/ultrasonic_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/line_tracking_launch.py'])
        ),

        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([
        #             FindPackageShare("adeept_awr_nodes"), '/launch', '/web_controller_launch.py'])
        # ),
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('rosbridge_server'),
        #             '/opt/ros/iron/share/rosbridge_server/launch/rosbridge_websocket_launch.xml'))
        # ),
    ])