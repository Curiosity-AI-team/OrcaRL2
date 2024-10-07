#!/bin/bash

# Run Python script
/bin/python3 ~/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/script/openrmf_convert.py new_map

# Remove build directories
sudo rm -r ~/orca_robot/colcon_ws/build/rmf_demos
sudo rm -r ~/orca_robot/colcon_ws/build/rmf_demos_maps
sudo rm -r ~/orca_robot/colcon_ws/build/rmf_demos_dashboard_resources

# Remove install directories
sudo rm -r ~/orca_robot/colcon_ws/install/rmf_demos
sudo rm -r ~/orca_robot/colcon_ws/install/rmf_demos_maps
sudo rm -r ~/orca_robot/colcon_ws/install/rmf_demos_dashboard_resources

# Build colcon workspace
cd ~/orca_robot/colcon_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --base-paths src

# Source setup.bash
source ~/orca_robot/colcon_ws/install/setup.bash
