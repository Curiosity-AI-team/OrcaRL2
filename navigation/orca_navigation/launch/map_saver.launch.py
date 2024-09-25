import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Declare map path argument
    declare_map_path = DeclareLaunchArgument(
        'map',
        default_value='my_map',
        description='Path to save the map'
    )

    # Map saver node
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        arguments=['-f', f"/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/{declare_map_path}/{declare_map_path}.pgm", '-t', '/map']
    )

    # Execute Python script after map_saver_cli completes
    register_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=map_saver_node,
            on_exit=[
                ExecuteProcess(
                    cmd=['python3', '/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/script/pgm2png.py', LaunchConfiguration("map")],
                    name='pgm_to_png',
                    output='screen'
                )
            ]
        )
    )

    return LaunchDescription([declare_map_path, map_saver_node, register_event_handler])