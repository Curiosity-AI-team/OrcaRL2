# OrcaRL2

```bash

vboxuser@Ubuntu20:~$ ros2 node info /boldbot_gazebo

/opt/ros/foxy/bin/ros2:6: DeprecationWarning: pkg_resources is deprecated as an API. See https://setuptools.pypa.io/en/latest/pkg_resources.html
  from pkg_resources import load_entry_point
/boldbot_gazebo
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /cm730/joint_commands: mx_joint_controller_msgs/msg/JointCommand
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /joint_states: sensor_msgs/msg/JointState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /boldbot_gazebo/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /boldbot_gazebo/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /boldbot_gazebo/get_parameters: rcl_interfaces/srv/GetParameters
    /boldbot_gazebo/list_parameters: rcl_interfaces/srv/ListParameters
    /boldbot_gazebo/set_parameters: rcl_interfaces/srv/SetParameters
    /boldbot_gazebo/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

```