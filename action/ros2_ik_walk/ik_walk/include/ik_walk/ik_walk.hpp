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

#ifndef IK_WALK__IK_WALK_HPP_
#define IK_WALK__IK_WALK_HPP_

#include "ik_walk/visibility_control.h"

#include <mx_joint_controller_msgs/msg/joint_command.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ik_walk_msgs/msg/walking_command.hpp"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include <vector>


#include "IKWalk/IKWalk.hpp"


namespace ik_walk
{

/**
 * Arm motor result positions
 */
struct ArmOutputs {
    double left_shoulder_pitch;
    double left_shoulder_roll;
    double right_shoulder_pitch;
    double right_shoulder_roll;
};

class IkWalkWrapper : public rclcpp::Node
{
public:
  IkWalkWrapper(
    const rclcpp::NodeOptions & options);

  virtual ~IkWalkWrapper();

private:
  using JointCommand = mx_joint_controller_msgs::msg::JointCommand;
  using Imu = sensor_msgs::msg::Imu;
  
  bool enabled;
  bool ramp_triggered;

  double phase_;
  double phase_previous_;
  double time_;
  double duration_;
  Eigen::Vector3d linear_acceleration_;
  Eigen::Vector3d angular_velocity_;

  // Timer for main command loop
  rclcpp::TimerBase::SharedPtr loopTimer_;
  struct Rhoban::IKWalkOutputs outputs_;
  struct Rhoban::IKWalkOutputs outputs_previous_;
  struct ArmOutputs arm_outputs_;

  #define WIN_SIZE 64 /* TODO: There may be other better values. */
  std::vector<double> movWinX;
  std::vector<double> movWinY;

  Rhoban::IKWalkOutputs clone_outputs ( Rhoban::IKWalkOutputs b);
  bool outputs_has_changed ( Rhoban::IKWalkOutputs a, Rhoban::IKWalkOutputs b);
  struct Rhoban::IKWalkParameters params_;

  rclcpp::Publisher<JointCommand>::SharedPtr jointCommandPub_;
  rclcpp::Subscription<Imu>::SharedPtr imuDataSub_;
  rclcpp::Subscription<ik_walk_msgs::msg::WalkingCommand>::SharedPtr walkingCommandSub_;
 
  std::tuple<float, float, float, float> calculate_params(const ik_walk_msgs::msg::WalkingCommand::SharedPtr msg);

  float clamp(float n, float lower, float upper);

  JointCommand::UniquePtr create_joint_command(const Rhoban::IKWalkOutputs & positions, ArmOutputs & arm_outputs);
  Rhoban::IKWalkParameters get_default_IKWalkParameters();
  ArmOutputs get_default_ArmOutputs();
  
  Rhoban::IKWalkOutputs simpleBalance(Rhoban::IKWalkOutputs walking_outputs);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsg);


  void walk(
    const Rhoban::IKWalkParameters & params, double & timeLength, double & phase,
    double & time);
};

}  // namespace ik_walk

#endif  // IK_WALK__IK_WALK_HPP_
