#!/bin/bash

MAP_NAME=$1

scp -P 22 robot@[2620:9b::1912:a92]:~/gr_platform/ros/src/localization/localization/2d_map/$MAP_NAME.pgm ~/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/$MAP_NAME.pgm
scp -P 22 robot@[2620:9b::1912:a92]:~/gr_platform/ros/src/localization/localization/2d_map/$MAP_NAME.yaml ~/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/$MAP_NAME.yaml

# also get update maps from all robots
ros2 launch orca_navigation map_saver.launch.py map:=$MAP_NAME