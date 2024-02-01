import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

gazebo_world_path = os.path.join(
      get_package_share_directory('gazebo_simulator_nodes'),
      'worlds',
      'world.sdf'
      )

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', gazebo_world_path],
            output='screen',
        ),
    ])