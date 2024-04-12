from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value="True")

    dc_motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("adeept_awr_nodes"), '/launch', '/dc_motor_launch.py'
        ]),
        condition=UnlessCondition(LaunchConfiguration("use_ros2_control"))
    )

    control_motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("adeept_awr_diffdrive_control_plugin"), '/launch', '/diffdrive_launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration("use_ros2_control"))
    )

    servo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/servo_launch.py'])
    )

    camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/pi_camera_launch.py'])
    )

    led = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/rgb_led_launch.py'])
    )

    ultrasonic =IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/ultrasonic_launch.py'])
    )

    imu =IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/imu_launch.py'])
    )

    line = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("adeept_awr_nodes"), '/launch', '/line_tracking_launch.py'])
    )

    sound_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("adeept_awr_nodes"), '/launch', '/sound_receiver_launch.py'
        ]),
        launch_arguments={'start_right_away': 'false', 'audio_stream_name':  '/control_transmitter_robot_receiver'}.items()
    )

    sound_transmitter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("adeept_awr_nodes"), '/launch', '/sound_transmitter_launch.py'
        ]),
        launch_arguments={'start_right_away': 'false', 'audio_stream_name':  '/robot_transmitter_control_receiver'}.items()
    )

    return LaunchDescription([
        use_ros2_control_arg,

        dc_motor,
        control_motor,
        
        servo,
        camera,
        led,
        ultrasonic,
        imu,
        line,
        sound_receiver,
        sound_transmitter,
    ])