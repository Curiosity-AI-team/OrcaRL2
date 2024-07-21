
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    icp_parameters={
          'odom_frame_id':'odom',
          'guess_frame_id':'odom',
          'qos':qos,
          'wait_imu_to_init': True,
    }

    rtabmap_parameters={
          'subscribe_rgbd':True,
          'subscribe_scan':True,
        #   'tag_detections':'/detections',
        #   'fiducial_transforms':'/fiducial_transforms',
          'use_action_for_goal':True,
          'odom_sensor_sync': True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Mem/NotLinkedNodesKept':'false'
    }

    # Shared parameters between different nodes
    shared_parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'Mem/NotLinkedNodesKept':'false',
    }

    remappings=[
          ('imu', '/imu/data_raw'),
          ('scan', '/points2'),
          ('rgb/image', '/camera_rgb/image_raw'),
          ('rgb/camera_info', '/camera_rgb/camera_info'),
          ('depth/image', '/camera_depth/depth/image_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'WARN']
            ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[icp_parameters, shared_parameters],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'WARN']
            ),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
            arguments=['-d', '--ros-args', '--log-level', 'WARN']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'WARN']
            ),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'WARN']
            ),
    ])