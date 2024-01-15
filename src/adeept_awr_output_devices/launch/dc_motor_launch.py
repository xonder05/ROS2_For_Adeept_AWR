import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='adeept_awr_output_devices',
            executable='dc_motor_node',
            name='dc_motor_node'),
  ])