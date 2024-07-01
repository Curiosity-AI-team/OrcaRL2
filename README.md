# OrcaRL2

ros2 launch boldbot_sim boldbot_sim_nav.launch.py

ros2 launch aruco_detect aruco_detect.launch.py

ros2 launch fiducial_slam fiducial_slam.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 topic echo /aruco_detect/fiducial_transforms

ros2 run tf2_tools view_frames.py


ros2 launch orca_rtabmap orca_rtabmap.launch.py use_sim_time:=true qos:=2

ros2 launch orca_rtabmap orca_rtabmap_slam.launch.py use_sim_time:=true qos:=2

ros2 launch orca_rtabmap orca_rtabmap_full.launch.py use_sim_time:=true qos:=2