import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'config',
        'config.yaml'
    )

    with open(config_file, 'r') as f:
        config_contents = yaml.safe_load(f)

    simulator_line_tracking = Node(
        package='gazebo_simulator_nodes',
        executable='simulator_line_tracking',
        parameters=[config_file]
    )
    
    #ros2 run ros_gz_bridge parameter_bridge /line_tracking@sensor_msgs/msg/Image@gz.msgs.Image
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            config_contents["simulator_line_tracking_bridge"]["ros__parameters"]["topic"] + '@' + 
            config_contents["simulator_line_tracking_bridge"]["ros__parameters"]["ros_message_type"] + '[' + 
            config_contents["simulator_line_tracking_bridge"]["ros__parameters"]["gazebo_message_type"]
        ]
    )

    return launch.LaunchDescription([
        simulator_line_tracking,
        bridge,
    ])