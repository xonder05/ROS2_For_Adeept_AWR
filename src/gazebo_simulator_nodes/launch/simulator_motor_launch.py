import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
      get_package_share_directory('gazebo_simulator_nodes'),
      'config',
      'config.yaml'
      )

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_simulator_nodes',
            executable='simulator_motor',
            name='simulator_motor',
            parameters=[config]
            ),
  ])