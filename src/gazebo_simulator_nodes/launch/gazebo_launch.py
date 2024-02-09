import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    gazebo_world_path = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'worlds',
        'world.sdf'
    )

    simulator = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r',  gazebo_world_path],
        output='screen',
    )

    return LaunchDescription([
        simulator
    ])