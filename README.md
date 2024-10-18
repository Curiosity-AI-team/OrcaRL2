# OrcaRL2

TODO:

- Fix the rtabmap slam navigation problem
- Implement in gazebo from dirrect control cmd_vel to robot joints
- Implement rasa_ai communication
- Add point cloud2 concatenation
- wrong free fleet behavior when robot exiting task

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

ros2 launch orca_rtabmap orca_rtabmap_slam_full.launch.py use_sim_time:=true qos:=2
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
/bin/python3 ~/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/scripts/openrmf_convert.py new_map
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
ROS_DOMAIN_ID=1
ros2 launch boldbot_sim boldbot_sim_nav.launch.py
# or
# ros2 launch orca_control launch_robot.launch.py
ROS_DOMAIN_ID=1
ros2 launch rover_simulation launch_nav.launch.py
ros2 launch rover_simulation launch_sim.launch.py

ROS_DOMAIN_ID=1
ros2 launch orca_rtabmap orca_rtabmap_slam_rgbd.launch.py use_sim_time:=true qos:=2 localization:=true

ROS_DOMAIN_ID=1
ros2 launch orca_navigation navigation2.launch.py use_sim_time:=True

ROS_DOMAIN_ID=1
ros2 launch orca_free_fleet_client client.launch.xml
# or
# ros2 launch orca_free_fleet_client fake_client.launch.xml

ROS_DOMAIN_ID=0
ros2 launch orca_free_fleet_server server.launch.xml map:=new_map5

ROS_DOMAIN_ID=0
ros2 launch task_publisher rasa_task_publisher.launch.py
```

5) Run some tasks

Or, submit a task via CLI:

```bash
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time
```
