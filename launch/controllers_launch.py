from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    user_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("controllers"), '/launch', '/user_interface_launch.py'
        ])
    )

    keyboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("controllers"), '/launch', '/keyboard_node_launch.py'
        ]),
        launch_arguments={'start_right_away': 'false'}.items()
    )

    gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("controllers"), '/launch', '/gamepad_node_launch.py'
        ]),
        launch_arguments={'start_right_away': 'false'}.items()
    )

    wandering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("controllers"), '/launch', '/wandering_node_launch.py'
        ]),
        launch_arguments={'start_right_away': 'false'}.items()
    )

    line_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("controllers"), '/launch', '/line_following_node_launch.py'
        ]),
        launch_arguments={'start_right_away': 'false'}.items()
    )

    return LaunchDescription([
        user_interface,
        keyboard,
        gamepad,
        wandering,
        line_following,
    ])
