from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    #ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py'
        ]),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    #ros2 launch nav2_bringup navigation_launch.py
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py'
        ]),
    )
    
    return LaunchDescription([
        navigation2,
        slam_toolbox,
    ])