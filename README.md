# OrcaRL2

TODO:

- Fix the rtabmap slam navigation problem
- Update the server of the free fleet
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
```bash
# use rviz the see the map topic then

# This step is the beginning if you use remote robot
ros2 launch orca_navigation map_saver.launch.py map:=new_map
```
3) Use map to make fleet management data
```bash

# use traffic editor to make fleet
traffic-editor

# save the map inside ~/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/*

# rebuild the open_rmf_map again
python3 /home/vboxuser/orca_robot/docs/openrmf_convert.py new_map
sudo rm -r /home/vboxuser/orca_robot/colcon_ws/build/rmf_demos
sudo rm -r /home/vboxuser/orca_robot/colcon_ws/build/rmf_demos_maps
sudo rm -r /home/vboxuser/orca_robot/colcon_ws/build/rmf_demos_dashboard_resources

sudo rm -r /home/vboxuser/orca_robot/colcon_ws/install/rmf_demos
sudo rm -r /home/vboxuser/orca_robot/colcon_ws/install/rmf_demos_maps
sudo rm -r /home/vboxuser/orca_robot/colcon_ws/install/rmf_demos_dashboard_resources
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
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

Or, submit a task via CLI:

```bash
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time
```


todo: add fake client to make it in right place

integrate way to make easy to make map

add the way to generate rmf_demos_tasks