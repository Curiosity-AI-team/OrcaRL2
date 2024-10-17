import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare map path argument
    declare_map_path = DeclareLaunchArgument(
        'map',
        default_value='my_map',
        description='Path to save the map'
    )

    # map_data = PathJoinSubstitution([
    #     os.path.join(get_package_share_directory('orca_navigation'), '2d_map'),
    #     LaunchConfiguration('map')
    # ])

    map_data = PathJoinSubstitution([
        "/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map",
        LaunchConfiguration('map')
    ])

    # Map saver node
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        arguments=['-f', map_data, '-t', '/map']
    )

    python_path = PathJoinSubstitution([
        os.path.join(get_package_share_directory('orca_navigation'), 'scripts'),
        'pgm2png.py'
    ])

    # orca_path = get_package_share_directory('orca_navigation')
    orca_path = '/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation'

    # rmf_path = get_package_share_directory('rmf_demos_maps')
    rmf_path = '/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_maps/maps'

    # Execute Python script after map_saver_cli completes
    register_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=map_saver_node,
            on_exit=[
                ExecuteProcess(
                    cmd=[
                        'python3',
                        python_path,
                        LaunchConfiguration("map"),
                        orca_path,
                        rmf_path
                        ],
                    name='pgm_to_png',
                    output='screen'
                )
            ]
        )
    )

    return LaunchDescription([declare_map_path, map_saver_node, register_event_handler])