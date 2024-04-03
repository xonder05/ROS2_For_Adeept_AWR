from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    xacro_model_path = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'models',
        'robot.xacro'
    )
    robot_description = Command(['xacro ', str(xacro_model_path)])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
        ],
        remappings=[("/robot_description", "/controller_manager/robot_description")],  # Remap the topic
    )

    controller_config_file = os.path.join(
        get_package_share_directory('adeept_awr_diffdrive_control_plugin'),
        'config',
        'diffbot_controllers.yaml'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        robot_controller_spawner,
    ])
