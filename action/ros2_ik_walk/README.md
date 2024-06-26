# ik_walk

This package provides a wrapper node for the open-loop walk engine [IKWalk](https://github.com/Rhoban/IKWalk).

## Node

The node `ik_walk_node` computes and publishes MX joint controller messages as `/cm730/joint_commands` topic every 8 ms. The [ros2_cm730](https://gitlab.com/boldhearts/ros2_cm730) package subscribes to that topic.

## Changing walk parameters

Walk parameters can be changed with a parameter file. An [example file](https://gitlab.com/boldhearts/ros2_ik_walk/-/blob/master/config/example.yml) is shipped with this package and is installed to the share directory at `config/example.yml`.

Usage: Copy the example file to the location where you hold all parameters of your specific robot. Pass the file when running the executable:

    ros2 run ik_walk ik_walk_node __params:=/path/to/robot/specific/ik_walk/my_robot_parameters.yml

The example file contains all possible parameters for the walk (including comments, based on the original [list of parameters](https://github.com/Rhoban/IKWalk/blob/master/Docs/parameters.md)).

## Simple Balance

## Walk commands

Publish a high level walk command message on `/walking/command` topic with 
    
    ros2 topic pub --once /walking/command ik_walk_msgs/msg/WalkingCommand '{stepgain: 0.0, lateralgain: 0.0, turngain: 0.0, urgency: 0, enabledgain: 1}'
  
  - ``stepgain``: Foward length of each foot step in meters (default=0.02, maximum=0.1).
  - ``lateralgain``: lateral lenghth of each foot step in meters.
  - ``turngain``: Angular yaw rotation of each foot for each step in radians, positive is clockwise.
  - ``urgency``: scales the stepgain and lateral gain. 
  - ``enabledgain``: enables or disables walking. 0 is disabled, 1 is enabled.
