import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='adeept_awr_output_devices',
            executable='servo_node',
            name='servo_node'),
  ])