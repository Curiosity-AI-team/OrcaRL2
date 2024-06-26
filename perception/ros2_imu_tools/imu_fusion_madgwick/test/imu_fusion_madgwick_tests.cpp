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

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "imu_fusion_madgwick/imu_fusion_madgwick.hpp"

template<typename T, int N>
inline ::testing::AssertionResult VectorsEqual(
  Eigen::Matrix<T, N, 1> const & expected, Eigen::Matrix<T, N, 1> const & actual,
  const double delta = 0.00001)
{
  double d = (expected - actual).norm();
  if (d < delta) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure() << "Actual: " << actual.transpose()
                                         << ", expected: " << expected.transpose() << " d = " << d;
  }
}

inline ::testing::AssertionResult MatricesEqual(
  Eigen::MatrixXd const & expected, Eigen::MatrixXd const & actual, double delta = 0.000001)
{
  double d = (expected - actual).array().abs().sum();
  if (d < delta) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure() << "Actual: " << std::endl
                                         << actual << std::endl
                                         << "Expected: " << std::endl
                                         << expected << std::endl
                                         << " d = " << d;
  }
}

TEST(OrientationTests, construction)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  auto quaternion = orientation.getEigenQuaternion();

  EXPECT_EQ(1.0, quaternion.w());
  EXPECT_EQ(Eigen::Vector3d::Zero(), quaternion.vec());
}

TEST(OrientationTests, gyro_halfPiXHighFreq)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;

  for (unsigned i = 0; i < 512; ++i) {
    orientation.integrate(Eigen::Vector3d{0.5 * M_PI, 0, 0}, 1.0 / 512);
  }

  auto quaternion = orientation.getEigenQuaternion();

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()).matrix(), quaternion.matrix()));
}

TEST(OrientationTests, gyro_halfPiX)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;

  orientation.integrate(Eigen::Vector3d{0.5 * M_PI, 0, 0}, 1.0);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_halfPiY)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;

  orientation.integrate(Eigen::Vector3d{0, 0.5 * M_PI, 0}, 1.0);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_halfPiZ)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0, 0, 0.5 * M_PI}, 1.0);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_piX)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0.5 * M_PI, 0, 0}, 2.0);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_piY)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0, 0.5 * M_PI, 0}, 2.0);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_piZ)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0, 0, 0.5 * M_PI}, 2.0);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_quartPiX)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0.5 * M_PI, 0, 0}, .5);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_quartPiY)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0, 0.5 * M_PI, 0}, .5);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitY()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_quartPiZ)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0, 0, 0.5 * M_PI}, .5);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitZ()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_multiAxis)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{1.0, 1.0, 1.0}.normalized() * 2.0 / 3.0 * M_PI, 1.0);

  EXPECT_TRUE(
    MatricesEqual(
      Eigen::AngleAxisd(2.0 / 3.0 * M_PI, Eigen::Vector3d(1.0, 1.0, 1.0).normalized()).matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, gyro_zero)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  Eigen::Vector4d oldCoeffs = orientation.getEigenQuaternion().coeffs();
  orientation.integrate(Eigen::Vector3d::Zero(), 0.008);

  EXPECT_TRUE(MatricesEqual(oldCoeffs, orientation.getEigenQuaternion().coeffs()));
}

TEST(OrientationTests, gyro_chained)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  orientation.integrate(Eigen::Vector3d{0.5 * M_PI, 0, 0}, 1.0);
  orientation.integrate(Eigen::Vector3d{0, 0, 0.5 * M_PI}, 1.0);

  EXPECT_TRUE(
    MatricesEqual(
      (Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()))
      .matrix(),
      orientation.getEigenQuaternion().matrix()));
}

TEST(OrientationTests, merged_no_error)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  auto q0 = orientation.getEigenQuaternion();
  orientation.integrate(Eigen::Vector3d::Zero(), 1.0, Eigen::Vector3d{0.0, 0.0, 1.0}, 0.1);

  EXPECT_TRUE(VectorsEqual(q0.coeffs(), orientation.getEigenQuaternion().coeffs()));
}

TEST(OrientationTests, merged_turned_no_error)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;
  // Turn 90 deg around x axis
  orientation.integrate(Eigen::Vector3d{0.5 * M_PI, 0, 0}, 1.0);
  auto q1 = orientation.getEigenQuaternion();
  // Gravity is now along y axis
  orientation.integrate(Eigen::Vector3d::Zero(), 1.0, Eigen::Vector3d{0.0, 1.0, 0.0}, 0.1);

  EXPECT_TRUE(VectorsEqual(q1.coeffs(), orientation.getEigenQuaternion().coeffs()));
}

TEST(OrientationTests, merged_90deg_error)
{
  imu_fusion_madgwick::IMUFusionMadgwick orientation;

  // Gravity is now along y axis
  orientation.integrate(Eigen::Vector3d::Zero(), 1.0, Eigen::Vector3d{0.0, 1.0, 0.0}, 0.1);

  EXPECT_FLOAT_EQ(1.0, orientation.getEigenQuaternion().norm());
  EXPECT_LT(0.0, orientation.getEigenQuaternion().x());

  // Test convergence after 20 seconds
  for (unsigned i = 0; i < 10000; ++i) {
    orientation.integrate(Eigen::Vector3d::Zero(), 1.0 / 500, Eigen::Vector3d{0.0, 1.0, 0.0}, 0.1);
  }

  // Quaternion element is sin(theta / 2)
  EXPECT_NEAR(sin(.25 * M_PI), orientation.getEigenQuaternion().coeffs().x(), 0.00002);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
