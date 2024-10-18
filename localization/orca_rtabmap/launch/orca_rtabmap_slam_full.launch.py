from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    # Parameters for rtabmap node
    rtabmap_parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'use_action_for_goal': True,
        'qos_scan_cloud': qos,
        'qos_imu': qos,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'True',
        'Grid/RangeMin': '0.2',  # Ignore points on the robot itself
        'Optimizer/GravitySigma': '0'  # Disable IMU constraints (we are already in 2D)
    }
    
    rtabmap_remappings = [
        ('scan_cloud', '/points2_scan')
    ]

    icp_odom_remappings = [
        ('scan_cloud', '/points2_scan')
    ]

    # Parameters for icp_odometry node
    icp_odom_parameters = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'wait_for_transform': 0.2,
        'use_sim_time': use_sim_time,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'qos_scan_cloud': qos,
        'Odom/ResetCountdown': '10',
    }

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # PointCloud filtering

        ComposableNodeContainer(
            name='box_filter_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # Box Filter Node
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::CropBox',
                    name='box_filter',
                    parameters=[{
                        'max_x': 30.0,
                        # 'max_y': -0.8,
                        'max_y': 30.0,
                        'max_z': 30.0,
                        'min_x': -30.0,
                        'min_y': -30.0,
                        'min_z': -30.0,
                    }],
                    remappings=[
                        ('input', '/camera_depth/points'),
                        ('output', '/points2_scan'),
                    ],
                ),
            ],
            output='screen',
        ),

        Node(
            package='points_concat_filter',
            executable='points_concat_async_node',
            name='points_concat_async_node',
            output='screen',
            # parameters=[icp_odom_parameters],
            # remappings=icp_odom_remappings,
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # # ICP Odometry Node
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[icp_odom_parameters],
            remappings=icp_odom_remappings,
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # RTAB-Map SLAM Node (Mapping Mode)
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remappings,
            arguments=['-d']
        ),
        
        # RTAB-Map SLAM Node (Localization Mode)
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True'
            }],
            remappings=rtabmap_remappings
        ),

        # RTAB-Map Visualization Node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remappings
        ),
    ])
