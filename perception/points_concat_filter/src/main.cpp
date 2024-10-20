#include "rclcpp/rclcpp.hpp"
#include "points_concat_async_node.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<perception::PointsConcatAsyncFilterNode>("points_concat_async_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}