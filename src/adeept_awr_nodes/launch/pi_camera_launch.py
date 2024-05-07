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

    pi_camera_node = Node(
        package='adeept_awr_nodes',
        executable='pi_camera_node',
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        pi_camera_node,
    ])