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

#ifndef IMU_FUSION_MADGWICK__IMU_FUSION_MADGWICK_HPP_
#define IMU_FUSION_MADGWICK__IMU_FUSION_MADGWICK_HPP_

#include <Eigen/Geometry>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace imu_fusion_madgwick
{

class IMUFusionMadgwick : public rclcpp::Node
{
public:
  IMUFusionMadgwick();

  virtual ~IMUFusionMadgwick();

  using Imu = sensor_msgs::msg::Imu;
  using Quaternion = geometry_msgs::msg::Quaternion;

  Quaternion getQuaternion() const;
  Eigen::Quaterniond getEigenQuaternion() const;

  void reset();
  void reset(const Quaternion & quat);
  void reset(const Eigen::Quaterniond & quat);

  /** A wrapper for the Eigen3 based integration implementation
   *
   * @param angular_velocity Measured angular velocity around axes, in rad/sec, e.g. from gyroscope
   * @param interval Time interval to integrate over, in seconds
   * @param linear_acceleration Measurement of the linear acceleration along axes, in meter/seconds^2
   * @param maxGyroError Maximum gyroscope measurement error, in rad/sec. Used to determine correction weight
   */
  void integrate(
    geometry_msgs::msg::Vector3 const & angular_velocity, double interval,
    geometry_msgs::msg::Vector3 const & linear_acceleration, double max_gyro_error);

  /** Integrate measurement of angular rate
   *
   * @param angularRate Measured angular rate around axes, in rad/sec, e.g. from gyroscope
   * @param interval Time interval to integrate over, in secons
   */
  void integrate(Eigen::Vector3d const & angularRate, double interval);

  /** Integrate measurement of angular rate, corrected given a reference measurement
   *
   * It uses the technique from paper:
   * "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
   * Sebastian O.H. Madgwick, April 30, 2010
   *
   * @param angularRate Measured angular rate around axes, in rad/sec, e.g. from gyroscope
   * @param interval Time interval to integrate over, in seconds
   * @param referenceDirMeas Measurement of the reference directory in local reference frame
   * @param maxGyroError Maximum gyroscope measurement error, in rad/sec. Used to determine correction weight
   * @param referenceDir Reference direction that is measured in global reference frame. By default direction of gravity
   */
  void integrate(
    Eigen::Vector3d const & angular_rate, double interval,
    Eigen::Vector3d const & reference_dir_measures, double max_gyro_error,
    Eigen::Vector3d const & referenceDir = Eigen::Vector3d{0, 0, 1});
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // parameter
  bool use_fixed_dt_;
  float gyro_measuring_error_;

  double dt_;
  rclcpp::Time last_update_time_;

  rclcpp::Subscription<Imu>::SharedPtr sub_;
  rclcpp::Publisher<Imu>::SharedPtr pub_;

  Eigen::Quaterniond orientation_;
};

}  // namespace imu_fusion_madgwick

#endif  // IMU_FUSION_MADGWICK__IMU_FUSION_MADGWICK_HPP_
