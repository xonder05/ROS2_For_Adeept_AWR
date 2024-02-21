import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('controllers'),
        'config',
        'config.yaml'
    )

    wandering_node = Node(
        package='controllers',
        executable='wandering_node',
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        wandering_node,
    ])