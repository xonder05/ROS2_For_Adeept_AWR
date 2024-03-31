from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )
    
    config_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('controllers'),
            'config',
            'custom_mapper_params_online_async.yaml'
        )
    )

    #ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./custom_mapper_params_online_async.yaml 
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[LaunchConfiguration('slam_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        slam_toolbox,
    ])