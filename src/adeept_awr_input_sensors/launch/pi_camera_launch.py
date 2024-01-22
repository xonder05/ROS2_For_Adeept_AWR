import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
      get_package_share_directory('adeept_awr_input_sensors'),
      'config',
      'sensors_config.yaml'
      )

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='adeept_awr_input_sensors',
            executable='pi_camera_node',
            name='pi_camera_node',
            parameters=[config]
            ),
  ])