import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('adeept_awr_nodes'),
        'config',
        'config.yaml'
    )

    line_tracking_node = Node(
        package='adeept_awr_nodes',
        executable='line_tracking_node',
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        line_tracking_node,
    ])