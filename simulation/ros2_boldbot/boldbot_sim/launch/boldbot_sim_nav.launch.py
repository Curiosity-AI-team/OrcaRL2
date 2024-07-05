import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    model_path = "/usr/share/gazebo-11/models:/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/ros2_boldbot/boldbot_sim/models:/home/vboxuser/orca_robot/colcon_ws/install/boldbot_sim/share/boldbot_sim/..:/home/vboxuser/orca_robot/colcon_ws/install/boldbot_description/share/boldbot_description/.."
    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    urdf_prefix = get_package_share_directory("boldbot_description")
    # urdf_file = os.path.join(urdf_prefix, "urdf", "boldbot.urdf")
    urdf_file = "/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/ros2_boldbot/boldbot_description/urdf/boldbot_nav.urdf"

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    world_prefix = get_package_share_directory("boldbot_sim")
    world_file = os.path.join(world_prefix, "worlds", "navigation.world")
    # world_prefix = get_package_share_directory("tuw_gazebo")
    # world_file = os.path.join(world_prefix, "worlds", "aruco.world")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    world_file,
                ],
                output="screen",
                additional_env=env,
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "boldbot",
                    "-x",
                    "-1",
                    "-y",
                    "0",
                    "-z",
                    ".41",
                    "-b",
                    "-file",
                    urdf_file,
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            #     parameters=[{'use_sim_time': True}]
            # ),
            Node(
                package='simulation',
                executable='fake_gps.py',
                name='fake_gps',
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('aruco_detect'), 'launch', 'aruco_detect.launch.py'])
                )),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         PathJoinSubstitution([FindPackageShare('yolov8_bringup'), 'launch', 'yolov8_3d.launch.py'])
            #     ),
            #     launch_arguments={'model': 'yolov8m-pose.pt'}.items()
            #     ),
        ]
    )
