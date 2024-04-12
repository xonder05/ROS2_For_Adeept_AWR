from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():

    controller_config_file = os.path.join(
        get_package_share_directory('adeept_awr_diffdrive_control_plugin'),
        'config',
        'controllers.yaml'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
        remappings=[
            ('/diffbot_base_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/controller_manager/robot_description', '/robot_description')
        ]
    )

    delayed_robot_controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diffbot_base_controller"],
            )
        ]
    )

    return LaunchDescription([
        control_node,
        delayed_robot_controller_spawner,
    ])
