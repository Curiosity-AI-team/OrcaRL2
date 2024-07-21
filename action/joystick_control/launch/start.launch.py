from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # joy_params = os.path.join(get_package_share_directory('orca_control'),'config','joystick.yaml')

    joystick_control_node = Node(
        package='joystick_control',
        executable='joystick_control_node.py',
        name='joystick_control',
        output='screen',
        # parameters=[{'dbc_arg': "/home/rover/gr_platform2/colcon_ws/src/control/joystick_control/config/joystick.dbc"}],
        # remappings=[('joy_vel', LaunchConfiguration('cmd_vel_topic'))]
    )   

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joystick_control_node
        # twist_stamper       
    ])