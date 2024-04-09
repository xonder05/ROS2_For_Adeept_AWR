import launch
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    robot_controller_spawner = TimerAction(
        period=7.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diffbot_base_controller"],
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
        robot_controller_spawner,
        bridge,
    ])