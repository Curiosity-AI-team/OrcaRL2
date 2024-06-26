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

#include "imu_viz/imu_viz.hpp"

#include <string>
#include <vector>

namespace imu_viz
{

ImuVizPublisher::ImuVizPublisher()
: rclcpp::Node{"imu_viz_publisher"}
{
  pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/imu/viz", rclcpp::SensorDataQoS());

  //
  frame_id_ = declare_parameter("frame_id", std::string{"base_link"});
  scale_axes_ = declare_parameter("scale_axes_", 0.5);

  //
  RCLCPP_INFO(get_logger(), "Setup axes marker");

  std::vector<std::string> axes = {"X", "Y", "Z"};
  for (auto & name : axes) {
    Marker marker = baseAxisMarker(name);
    marker_array_.markers.push_back(marker);
  }

  sub_ = create_subscription<ImuData>(
    "/imu/data", rclcpp::SensorDataQoS(), [&](ImuData::SharedPtr imu_msg) {
      auto timeNow = now();

      for (auto & marker : marker_array_.markers) {
        marker.header.stamp = timeNow;
        marker.action = Marker::MODIFY;
        marker.pose.orientation = imu_msg->orientation;
      }

      RCLCPP_DEBUG(get_logger(), "Publish new message of MarkerArray");
      pub_->publish(marker_array_);
    });
}

ImuVizPublisher::~ImuVizPublisher()
{
  RCLCPP_INFO(get_logger(), "Delete axes marker");

  for (auto & marker : marker_array_.markers) {
    marker.action = Marker::DELETE;
    // TODO(marcus) set duration
  }

  pub_->publish(marker_array_);
}

ImuVizPublisher::Marker ImuVizPublisher::baseAxisMarker(std::string axis_name)
{
  geometry_msgs::msg::Point origin;
  // origin.x = 0;
  // origin.y = 0;
  // origin.z = 0;

  visualization_msgs::msg::Marker axis_marker;

  axis_marker.header.frame_id = frame_id_;
  axis_marker.header.stamp = now();
  axis_marker.id = 0;
  axis_marker.ns = "imu_viz/" + axis_name;
  axis_marker.type = Marker::ARROW;
  axis_marker.action = Marker::ADD;

  axis_marker.scale.x = 0.1 * scale_axes_;  // shaft diameter
  axis_marker.scale.y = 0.2 * scale_axes_;  // head diameter
  // axisMarker.scale.z = 0.1; // head length

  // set start/end point to origin
  axis_marker.points.push_back(origin);
  axis_marker.points.push_back(origin);

  axis_marker.color.a = 1;

  // change color: XYZ -> RGB
  // change endpoint
  if (axis_name == "X") {
    axis_marker.color.r = 1;
    axis_marker.points.at(1).x = 1. * scale_axes_;
  } else if (axis_name == "Y") {
    axis_marker.color.g = 1;
    axis_marker.points.at(1).y = 1. * scale_axes_;
  } else if (axis_name == "Z") {
    axis_marker.color.b = 1;
    axis_marker.points.at(1).z = 1. * scale_axes_;
  }

  return axis_marker;
}

}  // namespace imu_viz
