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
                    FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_motor_launch.py'])
            ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_camera_launch.py'])
            ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_camera_servo_launch.py'])
            ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_ultrasonic_launch.py'])
            ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("gazebo_simulator_nodes"), '/launch', 'simulator_line_tracking_launch.py'])
            ),
    ])