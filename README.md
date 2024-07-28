# OrcaRL2

Some snipset to run the project

```bash
ros2 launch boldbot_sim boldbot_sim_nav.launch.py

ros2 launch aruco_detect aruco_detect.launch.py

ros2 launch fiducial_slam fiducial_slam.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 topic echo /aruco_detect/fiducial_transforms




ros2 launch orca_rtabmap orca_rtabmap.launch.py use_sim_time:=true qos:=2 

ros2 launch orca_rtabmap orca_rtabmap_slam.launch.py use_sim_time:=true qos:=2

ros2 launch orca_rtabmap orca_rtabmap_full.launch.py use_sim_time:=true qos:=2


## Clone the project

git clone https://github.com/Curiosity-AI-team/orca_robot.git --recursive

rosdep install --from-paths src -y --ignore-src

sudo apt-get install python3-rosdep

sudo apt-get install ros-foxy-ament-cmake

colcon build --packages-select orca_navigation

git submodule add https://github.com/pal-robotics/aruco_ros.git localization/aruco_ros
git submodule add https://github.com/open-rmf/rmf_demos.git simulation/rmf_demos

git submodule add https://github.com/ros-drivers/velodyne colcon_ws/src/OrcaRL2/localization/velodyne


$ git config --global user.name "John Doe" $ git config --global user.email hejhe@gmail.com

git config --global user.name "FIRST_NAME LAST_NAME"
git config --global user.email "MY_NAME@example.com"

killall -9 gzserver
killall -9 gzclient


ros2 pkg create --build-type ament_cmake --node-name orca_rmf orca_rmf --dependencies std_msgs


ros2 run rmf_building_map_tools model_downloader rmf_demos_maps -s office


git clone https://github.com/osrf/gazebo_models

# Navigate to the cloned repository
cd gazebo_models

# Copy all models to the ~/.gazebo/models/ directory
mkdir -p ~/.gazebo/models
cp -r ./* ~/.gazebo/models/.

export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH


ros2 launch simulation launch_sim.launch.py

ros2 launch orca_rtabmap orca_rtabmap_slam.launch.py

ros2 launch orca_navigation navigation2.launch.py use_sim_time:=True
```