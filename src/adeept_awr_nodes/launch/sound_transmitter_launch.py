import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('adeept_awr_nodes'),
        'config',
        'sound_config.yaml'
    )

    start_right_away_arg = DeclareLaunchArgument(
        'start_right_away',
        default_value='True'
    )

    audio_stream_name_arg = DeclareLaunchArgument(
        'audio_stream_name',
        default_value='/audio_stream'
    )

    sound_transmitter_node = Node(
        package='adeept_awr_nodes',
        executable='sound_transmitter_node',
        parameters=[config_file, {'start_right_away': LaunchConfiguration('start_right_away'), 'audio_stream_name': LaunchConfiguration('audio_stream_name')}]
    )
    
    return launch.LaunchDescription([
        start_right_away_arg,
        audio_stream_name_arg,
        sound_transmitter_node,
    ])