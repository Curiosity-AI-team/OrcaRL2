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

#include "imu_fusion_madgwick/imu_fusion_madgwick.hpp"

#include <string>

#include <rclcpp/logging.hpp>

namespace imu_fusion_madgwick
{

IMUFusionMadgwick::IMUFusionMadgwick()
: rclcpp::Node{"imu_fusion_madgwick"},
  last_update_time_(now()),
  orientation_{Eigen::Quaterniond::Identity()}
{
  gyro_measuring_error_ =
    declare_parameter("gyro_measuring_error", 3.14159265358979f * (5.0f / 180.0f));
  RCLCPP_INFO(
    get_logger(), "Use parameter: gyro measuring error (%s)",
    std::to_string(gyro_measuring_error_).c_str());

  // gain is unused
  // float beta;
  // get_parameter_or_set("gyroMeasError", beta, sqrt(3.0f / 4.0f) * gyroMeasError);

  // static sampling periodin seconds
  use_fixed_dt_ = declare_parameter("use_fixed_dt", false);
  dt_ = declare_parameter("fixed_dt", 0.008);

  if (use_fixed_dt_) {
    RCLCPP_INFO(get_logger(), "Use parameter: fixed dt (%s)", std::to_string(dt_).c_str());
  }

  RCLCPP_INFO(get_logger(), "Initialise robot orientation");
  reset();

  pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "Subscribe to /imu/data_raw");
  sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw", rclcpp::SensorDataQoS(), [ = ](sensor_msgs::msg::Imu::SharedPtr imuMsg) {
      if (!use_fixed_dt_) {
        // TODO(scheunemann) auto time = imuRawMsg->header.stamp;
        dt_ = (now() - last_update_time_).seconds();
        last_update_time_ = now();
      }

      //
      integrate(imuMsg->angular_velocity, dt_, imuMsg->linear_acceleration, gyro_measuring_error_);

      // set new orientation
      imuMsg->orientation = getQuaternion();

      // indicate that message
      // has valid orientation
      imuMsg->orientation_covariance.at(0) = 0;

      RCLCPP_DEBUG(get_logger(), "Received raw IMU message and publish IMU message");
      pub_->publish(*imuMsg);
    });
}

IMUFusionMadgwick::~IMUFusionMadgwick() {}

geometry_msgs::msg::Quaternion IMUFusionMadgwick::getQuaternion() const
{
  Quaternion quat;

  quat.x = orientation_.x();
  quat.y = orientation_.y();
  quat.z = orientation_.z();
  quat.w = orientation_.w();

  return quat;
}

Eigen::Quaterniond IMUFusionMadgwick::getEigenQuaternion() const {return orientation_;}

void IMUFusionMadgwick::reset() {orientation_ = Eigen::Quaterniond::Identity();}

void IMUFusionMadgwick::reset(const Quaternion & quaternion)
{
  orientation_.w() = quaternion.w;
  orientation_.x() = quaternion.x;
  orientation_.y() = quaternion.y;
  orientation_.z() = quaternion.z;
}

void IMUFusionMadgwick::reset(const Eigen::Quaterniond & quat) {orientation_ = quat;}

void IMUFusionMadgwick::integrate(
  geometry_msgs::msg::Vector3 const & angular_velocity, double interval,
  geometry_msgs::msg::Vector3 const & linear_acceleration, double max_gyro_error)
{
  auto reference_dir_measures =
    Eigen::Vector3d{linear_acceleration.x, linear_acceleration.y, linear_acceleration.z};

  auto angular_rate = Eigen::Vector3d{angular_velocity.x, angular_velocity.y, angular_velocity.z};

  integrate(angular_rate, interval, reference_dir_measures, max_gyro_error);
}

void IMUFusionMadgwick::integrate(const Eigen::Vector3d & angular_rate, double interval)
{
  auto theta = angular_rate * interval / 2;
  auto theta_mag = theta.norm();
  if (theta_mag == 0) {
    return;
  }

  auto delta_quat = Eigen::Quaterniond{};
  delta_quat.w() = cos(theta_mag);
  delta_quat.vec() = sin(theta_mag) / theta_mag * theta;

  // T_A2^W = T_A1^W * T_A2^A1
  orientation_ = orientation_ * delta_quat;
}

void IMUFusionMadgwick::integrate(
  Eigen::Vector3d const & angular_rate, double interval,
  Eigen::Vector3d const & reference_dir_measures, double max_gyro_error,
  Eigen::Vector3d const & reference_dir_global)
{
  integrate(angular_rate, interval);

  double constexpr kSqrt34 = 0.866025403784;  // sqrt(0.75);
  auto beta = kSqrt34 * max_gyro_error;

  // The objective function is the difference between
  // expected and measured reference dir in local frame
  // The quaternion describes the orientation of the local frame in the global frame,
  // so a vector multiplied with it is transformed from local to global

  Eigen::Vector3d objective_function =
    orientation_.conjugate() * reference_dir_global - reference_dir_measures;

  auto q1 = orientation_.w();
  auto q2 = orientation_.x();
  auto q3 = orientation_.y();
  auto q4 = orientation_.z();

  auto jacobian =
    (Eigen::Matrix<double, 3, 4>() <<
    2 * q4, -2 * q1, 2 * q2, -2 * q3,
    2 * q1, 2 * q4, 2 * q3, 2 * q2,
    -4 * q2, -4 * q3, 0.0, 0.0   ).
    finished();

  Eigen::Vector4d normalized_objective_function_gradient =
    jacobian.transpose() * objective_function;

  if (normalized_objective_function_gradient.squaredNorm() > 1e-16) {
    normalized_objective_function_gradient.normalize();
  }

  orientation_.coeffs() -= beta * normalized_objective_function_gradient * interval;
  orientation_.normalize();
}

}  // namespace imu_fusion_madgwick
