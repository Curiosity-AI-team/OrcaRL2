# OrcaRL2

Some snipset to run the project

```bash
ros2 launch boldbot_sim boldbot_sim_nav.launch.py

ros2 launch aruco_detect aruco_detect.launch.py

ros2 launch fiducial_slam fiducial_slam.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 topic echo /aruco_detect/fiducial_transforms

ros2 run nav2_map_server map_saver_cli -f my_map

pip install roslibpy

sudo apt-get install ros-humble-rosbridge-server


ros2 launch orca_rtabmap orca_rtabmap.launch.py use_sim_time:=true qos:=2 

ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py use_sim_time:=true qos:=2

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

ros2 run tf2_tools view_frames


ros2 launch simulation launch_sim.launch.py

ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py

ros2 launch orca_navigation navigation2.launch.py use_sim_time:=True
```


```bash

ros2 launch task_publisher rasa_task_publisher.launch.py

ros2 launch boldbot_sim boldbot_sim_nav.launch.py
# ros2 launch orca_control launch_robot.launch.py

ros2 launch orca_free_fleet_server server.launch.xml

ros2 launch orca_free_fleet_client client.launch.xml
# ros2 launch ff_examples_ros2 fake_client.launch.xml

ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py use_sim_time:=true qos:=2

ros2 launch orca_navigation navigation2.launch.py use_sim_time:=True

ros2 run rtabmap_slam rtabmap --params | grep g2o
ros2 run rtabmap_odom rgbd_odometry --params
```

TODO:

- Fix the rtabmap slam navigation problem
- Add the pipeline from slam mapping -> localization with open rmf
- Implement in gazebo from dirrect control cmd_vel to robot joints
- Implement rasa_ai communication

1) Start mapping environment
```bash
ros2 launch boldbot_sim boldbot_sim_nav.launch.py
# or
# ros2 launch orca_control launch_robot.launch.py
ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py use_sim_time:=true qos:=2
```
2) Save map data
```bash
# use rviz the see the map topic then
ros2 run nav2_map_server map_saver_cli -f my_map -t /map
```
3) Use map to make fleet management data
```bash
# convert pgm to png
python3 /home/vboxuser/orca_robot/docs/pgm2png.py
# use traffic editor to make fleet
traffic-editor

# rebuild the open_rmf_map again
colcon build
```
4) Run the robot in localization mode and use fleet to run the task
```bash

ros2 launch boldbot_sim boldbot_sim_nav.launch.py
# or
# ros2 launch orca_control launch_robot.launch.py

ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py use_sim_time:=true qos:=2 localization:=true

ros2 launch orca_navigation navigation2.launch.py use_sim_time:=True

ros2 launch orca_free_fleet_client client.launch.xml

ros2 launch orca_free_fleet_server server.launch.xml

ros2 launch task_publisher rasa_task_publisher.launch.py
```