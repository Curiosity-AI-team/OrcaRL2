# OrcaRL2

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