#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

class CanPubSub : public rclcpp::Node {
public:
    CanPubSub() : Node("can_pubsub") {
        // Initialize publisher
        publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_topic_pub", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CanPubSub::publish_message, this));

        // Initialize subscriber
        subscription_ = this->create_subscription<can_msgs::msg::Frame>(
            "can_topic_sub", 10,
            std::bind(&CanPubSub::topic_callback, this, std::placeholders::_1));
    }

private:
    void publish_message() {
        auto message = can_msgs::msg::Frame();
        message.id = 0x123;
        message.data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        RCLCPP_INFO(this->get_logger(), "Publishing CAN Frame ID: '%d'", message.id);
        publisher_->publish(message);
    }

    void topic_callback(const can_msgs::msg::Frame::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received CAN Frame ID: '%d'", msg->id);
    }

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanPubSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
