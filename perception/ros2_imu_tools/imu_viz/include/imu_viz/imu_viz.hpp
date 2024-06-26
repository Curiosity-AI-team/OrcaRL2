// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMU_VIZ__IMU_VIZ_HPP_
#define IMU_VIZ__IMU_VIZ_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace imu_viz
{

class ImuVizPublisher : public rclcpp::Node
{
public:
  ImuVizPublisher();

  virtual ~ImuVizPublisher();

private:
  using ImuData = sensor_msgs::msg::Imu;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  rclcpp::Subscription<ImuData>::SharedPtr sub_;

  // parameter
  std::string frame_id_;
  double scale_axes_;

  MarkerArray marker_array_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;

  Marker baseAxisMarker(std::string axis_name);
};
}  // namespace imu_viz

#endif  // IMU_VIZ__IMU_VIZ_HPP_
