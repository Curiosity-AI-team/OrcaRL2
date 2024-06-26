import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of 'gazebo_ros' package
    gazebo_ros_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_launch_dir, '/gazebo.launch.py']),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Node to spawn the robot
    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-x', '0', '-y', '0', '-z', '1', '-file', '/path/to/model.sdf'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_spawner
    ])
