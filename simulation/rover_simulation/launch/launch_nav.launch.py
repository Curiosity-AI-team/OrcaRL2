import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # static_transform_publisher1 =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments=['10.0', '0.0', '0', '0', '0', '0', 'rmf_map', 'map'],
    #         parameters=[{'use_sim_time': True}]
    # )


    static_transform_publisher2 =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
    )


    nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("orca_navigation"),'launch','test_navigation2.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    package_name='orca_control' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    manual_control = Node(package="teleop_twist_keyboard", executable="teleop_twist_keyboard",
        remappings=[('/cmd_vel','/cmd_vel_joy')],
        output='screen',
        prefix = 'xterm -e',
    )

    twist_mux = Node(package="twist_mux",  executable="twist_mux",
            parameters=[os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml'), {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel_out')]
        )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'extra_gazebo_args': '--ros-args --params-file /home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rover_simulation/config/gazebo_params.yaml'
                        }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description','-entity', 'my_bot'], output='screen')


    diff_drive_spawner = Node(package="controller_manager", executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(package="controller_manager", executable="spawner",
        arguments=["joint_broad"],
    )

    # Launch them all!
    return LaunchDescription([
        # static_transform_publisher1,
        static_transform_publisher2,
        nav,
        rsp,
        manual_control,
        twist_mux,
        gazebo,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])
