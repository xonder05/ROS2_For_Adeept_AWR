import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    
    gazebo_world_path = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'worlds',
        'wandering_world.sdf'
    )

    xacro_model_path = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'models',
        'robot.xacro'
    )

    urdf_model_path = os.path.join(
        get_package_share_directory('gazebo_simulator_nodes'),
        'models',
        'robot.urdf'
    )

    simulator = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4',  gazebo_world_path],
        output='screen',
    )

    #xacro robot.xacro -o robot.urdf
    convert_xacro = ExecuteProcess(
        cmd=['xacro', xacro_model_path, '-o', urdf_model_path],
        output='screen',
    )

    #ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/daniel/ros2_ws/src/gazebo_simulator_nodes/models/robot.urdf", name: "urdf_model"'
    spawn_robot = ExecuteProcess(
        cmd=['ign', 'service', 
             '--service', '/world/wandering/create', 
             '--reqtype', 'ignition.msgs.EntityFactory', 
             '--reptype', 'ignition.msgs.Boolean', 
             '--timeout', '5000', 
             '--req', f'sdf_filename: "{urdf_model_path}", name: "adeept_awr"'],
        output='screen',
    )

    remove_urdf = ExecuteProcess(
        cmd=['rm', urdf_model_path],
        output='screen',
    )

    simulator_start_event = RegisterEventHandler(
        OnProcessStart(
            target_action=simulator,
            on_start=convert_xacro
        )
    )

    urdf_file_creation_event = RegisterEventHandler(
        OnProcessExit(
            target_action=convert_xacro,
            on_exit=spawn_robot
        )
    )

    simulation_exit_event = RegisterEventHandler(
        OnProcessExit(
            target_action=simulator,
            on_exit=remove_urdf
        )
    )

    return LaunchDescription([
        simulator,
        simulator_start_event,
        urdf_file_creation_event,
        simulation_exit_event,
    ])