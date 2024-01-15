import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='adeept_awr_input_sensors',
            executable='pi_camera_node',
            name='pi_camera_node'),
  ])