import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('controllers'),
        'config',
        'config.yaml'
    )

    motor_controller_node = Node(
        package='controllers',
        executable='motor_controller_node',
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        motor_controller_node,
    ])