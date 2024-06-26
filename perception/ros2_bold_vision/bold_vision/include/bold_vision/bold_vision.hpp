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

#ifndef BOLD_VISION__BOLD_VISION_HPP_
#define BOLD_VISION__BOLD_VISION_HPP_

#include "bold_vision/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace bold_vision
{

class BoldVision : public rclcpp::Node
{
public:
  BoldVision();

  virtual ~BoldVision();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segmentation_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ball_pixels_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_position_pub_;
};

}  // namespace bold_vision

#endif  // BOLD_VISION__BOLD_VISION_HPP_
