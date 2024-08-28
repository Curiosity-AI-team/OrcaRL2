from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='task_publisher',
            executable='task_publisher.py',
            name='task_publisher_node',
            output='screen'
            # parameters=[{'use_sim_time': use_sim_time}],
            # arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
            ),
    ])