from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_imu':qos,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    
    remappings=[
          ('scan', '/scan')]
    
    icp_odom_remappings = [
        ('scan', '/scan')
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

        # Nodes to launch

        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[icp_odom_parameters],
            remappings=icp_odom_remappings,
            arguments=['--ros-args', '--log-level', 'info']
        ),


        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),
    ])