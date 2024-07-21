import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    socketcan_receive = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros2_socketcan'),'launch','socket_can_receiver.launch.py')
                    ])
    )

    socketcan_send = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros2_socketcan'),'launch','socket_can_sender.launch.py')
                    ])
    )

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    test_socketcan = Node(
            package="driver_odrive",
            executable="test_socketcan",
            # parameters=[twist_mux_params],
            remappings=[('can_topic_pub','/to_can_bus'), ('can_topic_sub','/from_can_bus')]
        )


    # Launch them all!
    return LaunchDescription([
        socketcan_receive,
        socketcan_send,
        test_socketcan,
        # delayed_controller_manager,
        # delayed_diff_drive_spawner,
        # delayed_joint_broad_spawner
    ])