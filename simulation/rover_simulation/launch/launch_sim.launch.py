import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'orca_control'  # <--- CHANGE ME

    # Add model path
    model_path = "/usr/share/gazebo-11/models:/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/ros2_boldbot/boldbot_sim/models:/home/vboxuser/orca_robot/colcon_ws/install/boldbot_sim/share/boldbot_sim/..:/home/vboxuser/orca_robot/colcon_ws/install/boldbot_description/share/boldbot_description/.."

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    manual_control = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        output='screen',
        prefix='xterm -e',
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            os.path.join(
                get_package_share_directory(package_name),
                'config',
                'twist_mux.yaml'
            ),
            {'use_sim_time': True}
        ],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Specify the custom world file
    # custom_world = os.path.join(
    #     get_package_share_directory('rover_simulation'),
    #     'worlds',
    #     'obstacles.world'  # Replace with your actual world file name
    # )

    custom_world = os.path.join(
        get_package_share_directory('boldbot_sim'),
        'worlds',
        'navigation.world'  # Replace with your actual world file name
    )
    
    # Include the Gazebo launch file with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': custom_world,
            'extra_gazebo_args': '--ros-args --params-file /home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rover_simulation/config/gazebo_params.yaml'
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    laser_to_pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud_node',
        remappings=[
            ('scan_in', 'scan'),
            ('cloud', 'points2_scan')
        ],
        output='screen'
    )

    # this package is not work well
    concat_pointcloud_node = Node(
        package='points_concat_filter',
        executable='points_concat_async_node',
        name='points_concat_async_node',
        remappings=[
            ('front_points', '/camera_depth/points'),
            ('left_points', '/points2_scan')
        ],
        output='screen'
    )


    # Launch all nodes with the model path set
    return LaunchDescription([
        set_gazebo_model_path,  # Set the custom model path environment variable
        rsp,
        manual_control,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        laser_to_pointcloud_node,
        # concat_pointcloud_node
    ])
