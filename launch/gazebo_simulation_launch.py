from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    world_select_arg = DeclareLaunchArgument('world_select', default_value='wandering')
    world_select_val = LaunchConfiguration('world_select')

    main_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/gazebo_garden_world_launch.py'
        ]),
        condition=IfCondition(
            PythonExpression([
                '"', world_select_val, '"', ' == "garden"'
            ])
        )
    )

    wandering_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/gazebo_wandering_world_launch.py'
        ]),
        condition=IfCondition(
            PythonExpression([
                '"', world_select_val, '"', ' == "wandering"'
            ])
        )
    )

    line_following_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/gazebo_line_tracking_world_launch.py'
        ]),
        condition=IfCondition(
            PythonExpression([
                '"', world_select_val, '"', ' == "line_tracking"'
            ])
        )
    )

    ros2_controlers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/ros2_controlers_launch.py'
        ])
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_camera_launch.py'
        ])
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_servo_launch.py'
        ])
    )

    ultrasonic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_ultrasonic_launch.py'
        ])
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_imu_launch.py'
        ])
    )

    line_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_line_tracking_launch.py'
        ])
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_simulator_nodes"), '/launch', '/simulator_lidar_launch.py'
        ])
    )

    return LaunchDescription([
        world_select_arg,
        main_world,
        wandering_world,
        line_following_world,
        
        ros2_controlers, #diff drive, odometry, joint_states
        servo,
        camera,
        line_following,
        ultrasonic,
        imu,
        lidar,
    ])