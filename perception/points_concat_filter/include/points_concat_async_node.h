#ifndef POINTS_CONCAT_ASYNC_NODE_H
#define POINTS_CONCAT_ASYNC_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

// PCL library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <vector>

namespace perception {

class PointsConcatAsyncFilterNode : public rclcpp::Node {

public:
  explicit PointsConcatAsyncFilterNode(const std::string &node_name);
  ~PointsConcatAsyncFilterNode();

private:
  void frontCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void leftCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void rightCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timerCallback();

  void makeTransformList();
  void transformPointCloudCustom(
      const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
      pcl::PointCloud<pcl::PointXYZI> &cloud_out,
      const tf2::Transform &transform);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_front_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_left_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_right_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_concat_points_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Flags and storage
  rclcpp::Time front_last_update_;
  rclcpp::Time left_last_update_;
  rclcpp::Time right_last_update_;
  rclcpp::Duration timeout_;

  bool b_front_;
  bool b_left_;
  bool b_right_;
  bool b_transform_list_generated_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_front_points_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_left_points_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_right_points_;

  std::vector<tf2::Transform> transform_list_;
};

} // namespace perception

#endif // POINTS_CONCAT_ASYNC_NODE_H