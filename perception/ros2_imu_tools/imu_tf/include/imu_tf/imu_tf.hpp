// Copyright 2020 Bold Hearts
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

#ifndef IMU_TF__IMU_TF_HPP_
#define IMU_TF__IMU_TF_HPP_

#include <geometry_msgs/msg/quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

namespace imu_tf
{

class IMUTF : public rclcpp::Node
{
public:
  IMUTF();

  virtual ~IMUTF();

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  void publish_tf(
    const sensor_msgs::msg::Imu imuMsg, std::string const & source_frame,
    std::string const & target_frame);

  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace imu_tf

#endif  // IMU_TF__IMU_TF_HPP_
