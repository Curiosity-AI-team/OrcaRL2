# IMU tools
This repository provides packages for computing and visualizing the orientation of, e.g., a robot.

## Components

### `imu_fusion_madgwick`

This package implements a filter for fusing accelerometer and gyrometer readings to orientation. It subscribes to the topic `/imu/data_raw` and publishes the computed orientation to `/imu/data` (including the raw accelerometer and raw gyrometer readings).

### `imu_tf`

Provides a node `transform` which transforms latest `/tf` to world frame coordinates using the orientation retrieved from the subscribed topic `/imu/data` and then broadcasts the transformed `/tf`.

Parameters: 
- `source_frame` - `string`, default: `"imu_link"`
- `target_frame` - `string`, default: `"imu_link_oriented"`

### `imu_viz`

This package visualizes the orientation of the robot with three arrows representing the axes X (roll), Y (pitch) and Z (yaw). The axes (XYZ) are mapped to the colors red, green and blue (RGB).

For seeing the axes in RViz, you need to add a Display of type `MarkerArray` and subscibe to the topic `/imu/viz`.

![The initial axes of the IMU visualization](https://gitlab.com/boldhearts/ros2_imu_tools/raw/master/imu_viz_marker_array.png)