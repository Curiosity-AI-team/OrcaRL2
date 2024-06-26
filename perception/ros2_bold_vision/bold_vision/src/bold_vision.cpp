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

#include "bold_vision/bold_vision.hpp"

#include <numeric>
#include <vector>
#include <cv_bridge/cv_bridge.h>

namespace bold_vision
{

BoldVision::BoldVision()
: rclcpp::Node{"bold_vision"}
{
  ball_pixels_pub_ = create_publisher<sensor_msgs::msg::Image>("ball_pixels", 10);
  ball_position_pub_ = create_publisher<geometry_msgs::msg::Point>("ball_position", 10);

  segmentation_sub_ =
    create_subscription<sensor_msgs::msg::Image>("/tflite/output", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        auto shared_msg = std::shared_ptr<sensor_msgs::msg::Image>{std::move(msg)};
        auto cv_img = cv_bridge::toCvShare(shared_msg);

        auto ball_channel = cv_bridge::CvImage{};
        cv::extractChannel(cv_img->image, ball_channel.image, 1);
        cv::threshold(ball_channel.image, ball_channel.image, 0.8 * 255, 255, CV_THRESH_BINARY);

        auto components = cv::Mat{};
        auto stats = cv::Mat{};
        auto centroids = cv::Mat{};
        auto n_components =
        cv::connectedComponentsWithStats(ball_channel.image, components, stats, centroids);

        auto component_idxs = std::vector<int>(n_components - 1);
        std::iota(component_idxs.begin(), component_idxs.end(), 1);

        auto idxs_end = component_idxs.end();

        // filter out blobs that are too small
        idxs_end = std::remove_if(component_idxs.begin(), idxs_end,
        [&](int idx) {
          return stats.at<int>(idx, cv::CC_STAT_AREA) < 25;
        });

        // filter out blobs that are not square enough
        idxs_end = std::remove_if(component_idxs.begin(), idxs_end,
        [&](int idx) {
          auto ratio =
            double(stats.at<int>(idx, cv::CC_STAT_WIDTH))
            / double(stats.at<int>(idx, cv::CC_STAT_HEIGHT));
          ratio = std::max(ratio, 1.0 / ratio);

          return ratio > 5.0;
        });

        if (idxs_end != component_idxs.begin()) {
          // Find the nearest (approximation: lowest in the image/highest row number)
          auto nearest = std::max_element(component_idxs.begin(), idxs_end,
            [&](int idx1, int idx2) {
              return centroids.at<double>(idx1, 1) < centroids.at<double>(idx2, 1);
            });

          auto ball_position = geometry_msgs::msg::Point{};
          ball_position.x = centroids.at<double>(*nearest, 0);
          ball_position.y = centroids.at<double>(*nearest, 1);
          ball_position_pub_->publish(ball_position);
        }
        auto out_img = ball_channel.toImageMsg();
        out_img->encoding = "mono8";
        // ball_pixels_pub_->publish(out_img);
      });
}

BoldVision::~BoldVision()
{
}

}  // namespace bold_vision
