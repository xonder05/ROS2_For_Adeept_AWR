from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/dc_motor_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/servo_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/pi_camera_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/rgb_led_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/ultrasonic_launch.py'])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("adeept_awr_nodes"), '/launch', '/line_tracking_launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/sound_receiver_launch.py'
            ]),
            launch_arguments={'start_right_away': 'false', 'audio_stream_name':  '/control_transmitter_robot_receiver'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/sound_transmitter_launch.py'
            ]),
            launch_arguments={'start_right_away': 'false', 'audio_stream_name':  '/robot_transmitter_control_receiver'}.items()
        )
    ])