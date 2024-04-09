from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    controller_config_file = os.path.join(
        get_package_share_directory('adeept_awr_diffdrive_control_plugin'),
        'config',
        'diffbot_controllers.yaml'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        robot_controller_spawner,
    ])
