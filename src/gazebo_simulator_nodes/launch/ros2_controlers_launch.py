import launch
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    diff_drive_controller_spawner = TimerAction(
        period=7.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
            )
        ]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=7.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            )
        ]
    )

    #ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
        ]
    )

    return launch.LaunchDescription([
        diff_drive_controller_spawner,
        joint_state_broadcaster_spawner,
        bridge,
    ])