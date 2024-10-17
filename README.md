# OrcaRL2

TODO:

- Fix the rtabmap slam navigation problem
- Implement in gazebo from dirrect control cmd_vel to robot joints
- Implement rasa_ai communication

For more info:
https://dds-demonstrators.readthedocs.io/en/latest/Teams/1.Hurricane/DDSWan.html
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>ham0</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        <FragmentSize>4000B</FragmentSize>
        <Transport>udp6</Transport>
        </General>
    <Discovery>
        <Peers>
            <Peer address="2620:9b::1921:60c"/>
            <Peer address="2620:9b::1912:a92"/>
        </Peers>
        <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
        <Tracing>
            <Verbosity>severe</Verbosity>
            <OutputFile>stdout</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```


1) Start mapping environment
```bash
ros2 launch boldbot_sim boldbot_sim_nav.launch.py
# or
# ros2 launch orca_control launch_robot.launch.py
ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py use_sim_time:=true qos:=2
```


2) Save map data

This step is the beginning if you use remote robot
Use rviz the see the map topic then
```bash
~/orca_robot/colcon_ws/src/OrcaRL2/operation/shell_scripts/map_updater.sh new_map
```

Alternative:
```bash
scp -P 22 robot@[2620:9b::1912:a92]:~/gr_platform/ros/src/localization/localization/2d_map/new_map.pgm ~/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/new_map.pgm
scp -P 22 robot@[2620:9b::1912:a92]:~/gr_platform/ros/src/localization/localization/2d_map/new_map.yaml ~/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/new_map.yaml

ros2 launch orca_navigation map_saver.launch.py map:=new_map
```


3) Use map to make fleet management data

save the map inside `~/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/<>`
Use traffic editor to make fleet
```bash
traffic-editor
```

```bash
~/orca_robot/colcon_ws/src/OrcaRL2/operation/shell_scripts/map_builder.sh new_map
```

Alternative:
```bash
# rebuild the open_rmf_map again
/bin/python3 ~/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/script/openrmf_convert.py new_map
sudo rm -r ~/orca_robot/colcon_ws/build/rmf_demos
sudo rm -r ~/orca_robot/colcon_ws/build/rmf_demos_maps
sudo rm -r ~/orca_robot/colcon_ws/build/rmf_demos_dashboard_resources

sudo rm -r ~/orca_robot/colcon_ws/install/rmf_demos
sudo rm -r ~/orca_robot/colcon_ws/install/rmf_demos_maps
sudo rm -r ~/orca_robot/colcon_ws/install/rmf_demos_dashboard_resources
cd ~/orca_robot/colcon_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --base-paths src
source ~/orca_robot/colcon_ws/install/setup.bash
```
rtabmap -i /home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/3d_map/office.pcd

4) Run the robot in localization mode and use fleet to run the task
```bash

ros2 launch boldbot_sim boldbot_sim_nav.launch.py
# or
# ros2 launch orca_control launch_robot.launch.py

ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py use_sim_time:=true qos:=2 localization:=true

ros2 launch orca_navigation navigation2.launch.py use_sim_time:=True

ros2 launch orca_free_fleet_client client.launch.xml
# or
# ros2 launch orca_free_fleet_client fake_client.launch.xml

ros2 launch orca_free_fleet_server server.launch.xml map:=new_map

ros2 launch task_publisher rasa_task_publisher.launch.py
```

5) Run some tasks

Or, submit a task via CLI:

```bash
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time
```

TODO: 
- fix bug! Remove python to use relativa path
- Add point cloud concatenation
- test open rmf task

```bash

vboxuser@Ubuntu22:~$ ~/orca_robot/colcon_ws/src/OrcaRL2/operation/shell_scripts/map_updater.sh new_map3
ssh: connect to host 2620:9b::1912:a92 port 22: No route to host
ssh: connect to host 2620:9b::1912:a92 port 22: No route to host
[INFO] [launch]: All log files can be found below /home/vboxuser/.ros/log/2024-10-16-17-13-55-083182-Ubuntu22-57459
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [map_saver_cli-1]: process started with pid [57464]
[map_saver_cli-1] [INFO] [1729088035.310212974] [map_saver]: 
[map_saver_cli-1] 	map_saver lifecycle node launched. 
[map_saver_cli-1] 	Waiting on external lifecycle transitions to activate
[map_saver_cli-1] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[map_saver_cli-1] [INFO] [1729088035.312214166] [map_saver]: Creating
[map_saver_cli-1] [INFO] [1729088035.312259951] [map_saver]: Configuring
[map_saver_cli-1] [INFO] [1729088035.319665767] [map_saver]: Saving map from '/map' topic to '/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x7f39658af910>/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x7f39658af910>.pgm' file
[map_saver_cli-1] [WARN] [1729088035.319695913] [map_saver]: Free threshold unspecified. Setting it to default value: 0.250000
[map_saver_cli-1] [WARN] [1729088035.319703598] [map_saver]: Occupied threshold unspecified. Setting it to default value: 0.650000
[map_saver_cli-1] [ERROR] [1729088037.327274424] [map_saver]: Failed to spin map subscription
[map_saver_cli-1] [INFO] [1729088037.328082919] [map_saver]: Destroying
[ERROR] [map_saver_cli-1]: process has died [pid 57464, exit code 1, cmd '/opt/ros/humble/lib/nav2_map_server/map_saver_cli -f /home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x7f39658af910>/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x7f39658af910>.pgm -t /map --ros-args -r __node:=map_saver'].
[INFO] [pgm_to_png-2]: process started with pid [57482]
[pgm_to_png-2] MAP NAME is new_map3
[pgm_to_png-2] Traceback (most recent call last):
[pgm_to_png-2]   File "/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/script/pgm2png.py", line 24, in <module>
[pgm_to_png-2]     cv2.imwrite(output_path, input_img)
[pgm_to_png-2] cv2.error: OpenCV(4.5.4) ./modules/imgcodecs/src/loadsave.cpp:799: error: (-215:Assertion failed) !_img.empty() in function 'imwrite'
[pgm_to_png-2] 
[ERROR] [pgm_to_png-2]: process has died [pid 57482, exit code 1, cmd 'python3 /home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/script/pgm2png.py new_map3'].

vboxuser@Ubuntu22:~$ ~/orca_robot/colcon_ws/src/OrcaRL2/operation/shell_scripts/map_updater.sh new_map3
ssh: connect to host 2620:9b::1912:a92 port 22: No route to host
ssh: connect to host 2620:9b::1912:a92 port 22: No route to host
[INFO] [launch]: All log files can be found below /home/vboxuser/.ros/log/2024-10-16-17-14-53-460154-Ubuntu22-57728
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [map_saver_cli-1]: process started with pid [57732]
[map_saver_cli-1] [INFO] [1729088093.716607805] [map_saver]: 
[map_saver_cli-1] 	map_saver lifecycle node launched. 
[map_saver_cli-1] 	Waiting on external lifecycle transitions to activate
[map_saver_cli-1] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[map_saver_cli-1] [INFO] [1729088093.720363126] [map_saver]: Creating
[map_saver_cli-1] [INFO] [1729088093.723340639] [map_saver]: Configuring
[map_saver_cli-1] [INFO] [1729088093.727971281] [map_saver]: Saving map from '/map' topic to '/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x72c87af1b910>/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x72c87af1b910>.pgm' file
[map_saver_cli-1] [WARN] [1729088093.728011456] [map_saver]: Free threshold unspecified. Setting it to default value: 0.250000
[map_saver_cli-1] [WARN] [1729088093.728021334] [map_saver]: Occupied threshold unspecified. Setting it to default value: 0.650000
[map_saver_cli-1] [ERROR] [1729088095.737518742] [map_saver]: Failed to spin map subscription
[map_saver_cli-1] [INFO] [1729088095.739816260] [map_saver]: Destroying
[ERROR] [map_saver_cli-1]: process has died [pid 57732, exit code 1, cmd '/opt/ros/humble/lib/nav2_map_server/map_saver_cli -f /home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x72c87af1b910>/<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x72c87af1b910>.pgm -t /map --ros-args -r __node:=map_saver'].
[INFO] [pgm_to_png-2]: process started with pid [57745]
[pgm_to_png-2] MAP NAME is new_map3
[pgm_to_png-2] Done!
[INFO] [pgm_to_png-2]: process has finished cleanly [pid 57745]
vboxuser@Ubuntu22:~$ ^C

```