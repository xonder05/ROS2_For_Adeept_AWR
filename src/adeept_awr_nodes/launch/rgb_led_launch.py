import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='adeept_awr_nodes',
            executable='rgb_led_node',
            name='rgb_led_node'),
  ])