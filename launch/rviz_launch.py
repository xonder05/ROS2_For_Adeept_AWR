from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_path = os.path.join(
        get_package_share_directory('controllers'),
        'config',
        'config.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': True}
        ],
    )

    xacro_model_path = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'models',
        'robot.xacro'
    )

    robot_description = Command(['xacro ', str(xacro_model_path)])

    # ros2 run robot_state_publisher robot_state_publisher ./robot.urdf --ros-args -p use_sim_time:=true
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        rviz,
        robot_state_publisher,
    ])