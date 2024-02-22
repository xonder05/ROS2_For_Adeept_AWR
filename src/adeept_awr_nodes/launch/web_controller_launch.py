import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='adeept_awr_web_controller',
            executable='web_controller_node',
            name='web_controller_node'),
  ])