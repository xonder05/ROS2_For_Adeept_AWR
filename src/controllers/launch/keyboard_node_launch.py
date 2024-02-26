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

    start_right_away_arg = DeclareLaunchArgument(
        'start_right_away',
        default_value='True'
    )

    keyboard_node = Node(
        package='controllers',
        executable='keyboard_node',
        parameters=[config_file, {'start_right_away': LaunchConfiguration('start_right_away')}]
    )
    
    return launch.LaunchDescription([
        start_right_away_arg,
        keyboard_node,
    ])