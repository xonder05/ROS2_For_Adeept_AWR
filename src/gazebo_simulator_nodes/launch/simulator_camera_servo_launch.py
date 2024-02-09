import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'config',
        'config.yaml'
    )

    simulator_camera_servo = Node(
        package='gazebo_simulator_nodes',
        executable='simulator_camera_servo',
        #parameters=[config]
    )
    
    #ros2 run ros_gz_bridge parameter_bridge /model/adeept_awr/joint/camera_servo_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/adeept_awr/joint/camera_servo_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double']
    )

    return launch.LaunchDescription([
        simulator_camera_servo,
        bridge,
    ])