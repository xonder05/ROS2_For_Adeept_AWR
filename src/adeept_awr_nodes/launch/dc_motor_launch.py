import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
      get_package_share_directory('adeept_awr_nodes'),
      'config',
      'output_config.yaml'
      )

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='adeept_awr_nodes',
            executable='dc_motor_node',
            name='dc_motor_node',
            parameters=[config]
            ),
  ])