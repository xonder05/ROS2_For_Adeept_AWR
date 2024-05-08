from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_arg = DeclareLaunchArgument('use_sim', default_value="False")

    xacro_model_path = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'models',
        'robot.xacro'
    )

    robot_description = Command(['xacro ', str(xacro_model_path), ' use_sim:=', LaunchConfiguration("use_sim")])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration("use_sim")},
            {'robot_description': robot_description},
        ],
    )

    return LaunchDescription([
        use_sim_arg,
        robot_state_publisher,
    ])