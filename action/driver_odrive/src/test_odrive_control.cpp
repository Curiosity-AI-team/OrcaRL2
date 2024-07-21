#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include <iostream>
#include <chrono>
#include <map>
#include <vector>
#include <dbcppp/Network.h>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>

class CanPubSub : public rclcpp::Node {
public:
    CanPubSub() : Node("can_pubsub") {
        rclcpp::QoS custom_qos_profile(10);
        custom_qos_profile.avoid_ros_namespace_conventions(true);

        publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_topic_pub", custom_qos_profile);
        subscription_ = this->create_subscription<can_msgs::msg::Frame>("can_topic_sub", custom_qos_profile,
            std::bind(&CanPubSub::topic_callback, this, std::placeholders::_1));

        control_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "topic_name", 10, std::bind(&CanPubSub::control_callback, this, std::placeholders::_1));

        // Initialize variables
        current_time = std::chrono::system_clock::now();
        last_msg_time = std::chrono::system_clock::now();
        axis_list = {0, 1, 2, 3}; // Example axis list
        active_state = std::vector<int>(axis_list.size(), 0);
        buffer = std::vector<std::time_t>(axis_list.size(), 0);
        

    }

        
private:

    void control_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received twist message: Linear: x=%f, y=%f, z=%f Angular: x=%f, y=%f, z=%f",
                    msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    }


    void topic_callback(const can_msgs::msg::Frame::SharedPtr msg) {
        current_time = std::chrono::system_clock::now();
        auto message = can_msgs::msg::Frame();

        for (int i : axis_list) {
            if (msg->id == (static_cast<unsigned int>(i) << 5 | 0x01)) {
                int errorCode = msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) | (msg->data[3] << 24);
                if (errorCode != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Driver axis %d error code: %x", i, errorCode);
                    active_state[i] = 1;
                }
            }

            if (std::chrono::duration_cast<std::chrono::seconds>(current_time - std::chrono::system_clock::from_time_t(buffer[i])).count() > 1) {
                active_state[i] = 1;
            } else {
                if (active_state[i] == 1) {
                    active_state[i] = 0;
                    // Assuming db and bus are correctly defined and accessible

                    // const dbcppp::IMessage* dbcppp_msg = net->getMessageById(14);
                    // dbcppp::ISignal::signal_data_t data = 0;
                    // message.id = (0x18 | (i << 5));
                    // message.data = data;
                    // publisher_->publish(message);


                    // const dbcppp::IMessage* dbcppp_msg = net->getMessageById(7);
                    // dbcppp::ISignal::signal_data_t data = 0;
                    // dbcppp_msg->encode(data, "Axis_Requested_State", 0x08);
                    // message.id = (0x07 | (i << 5));
                    // message.data = data;
                    // publisher_->publish(message);

                    RCLCPP_INFO(this->get_logger(), "Clearing errors and setting axis state for axis %d", i);
                }
            }
        }

        if ((msg->id & 0x1F) == 0x09) {
            int axis_id = msg->id >> 5;
            buffer[axis_id] = std::chrono::system_clock::to_time_t(current_time);

            // Assuming db and bus are correctly defined and accessible
            // auto data = db.decode_message("Get_Encoder_Estimates", msg->data);
            // float v_cur = data["Vel_Estimate"];

            if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_msg_time).count() > 1) {
                wheel = 0;
            } else {
                if (axis_id % 2 == 0) {
                    wheel = vel_lin_setpoint - vel_ang_setpoint * l_distance / 2;
                } else {
                    wheel = vel_lin_setpoint + vel_ang_setpoint * l_distance / 2;
                }
            }
            

            //  // Create a message by ID - assuming 13 is the ID for Set_Input_Vel
            // const dbcppp::IMessage* dbcppp_msg = net->getMessageById(13);
            // dbcppp::ISignal::signal_data_t data = 0;
            // uint64_t input_vel_value = 2;      // Example value for Input_Vel
            // uint64_t input_torque_ff_value = 0; // Example value for Input_Torque_FF
            // dbcppp_msg->encode(data, "Input_Vel", input_vel_value);
            // dbcppp_msg->encode(data, "Input_Torque_FF", input_torque_ff_value);

            message.id = (axis_id << 5 | 0x00D);
            // message.data = data;
            publisher_->publish(message);

            if (DEBUG) {
                RCLCPP_INFO(this->get_logger(), "Received CAN Frame ID: '%d'", msg->id);
            }
        }

    }
    
    double l_distance = 1;
    double vel_lin_setpoint = 0;
    double vel_ang_setpoint = 0;
    double wheel = 0;
    bool arm_msg = true;
    const bool DEBUG = false;
    std::vector<int> axis_list;
    std::vector<int> active_state;
    std::vector<std::time_t> buffer;
    std::chrono::time_point<std::chrono::system_clock> current_time, last_msg_time;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;

};

int main(int argc, char* argv[]) {

    std::unique_ptr<dbcppp::INetwork> net;
    {
        std::ifstream idbc( "/home/rover/gr_platform2/colcon_ws/src/control/driver_odrive/config/odrive-cansimple.dbc");
        net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }

    std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;
    for (const dbcppp::IMessage& msg : net->Messages())
    {   
        std::cout << "Hello" << "\n";
        messages.insert(std::make_pair(msg.Id(), &msg));
    }
    
    // Example: Set Input Velocity
    // auto set_input_vel = network->getMessageByName("Set_Input_Vel");
    // auto input_vel_signal = set_input_vel->getSignalByName("Input_Vel");
    // std::vector<uint8_t> data(set_input_vel->getMessageSize(), 0);
    // input_vel_signal->encode(data.data(), 1000.0);  // Set velocity to 1000 units

    // // Example: Clear Errors
    // auto clear_errors = network->getMessageByName("Clear_Errors");
    // // Assuming no signals need to be set for clearing errors

    // // Example: Set Axis State
    // auto set_axis_state = network->getMessageByName("Set_Axis_State");
    // auto axis_state_signal = set_axis_state->getSignalByName("Axis_Requested_State");
    // axis_state_signal->encode(data.data(), 1);  // Set state to 1 (example state)

    // // Example: Get Encoder Estimates
    // auto get_encoder_estimates = network->getMessageByName("Get_Encoder_Estimates");
    // auto vel_estimate_signal = get_encoder_estimates->getSignalByName("Vel_Estimate");
    // auto pos_estimate_signal = get_encoder_estimates->getSignalByName("Pos_Estimate");
    // // Assuming you need to read these values from a response

    // // Print encoded data for demonstration
    // std::cout << "Set Input Vel Data: ";
    // for (auto byte : data) {
    //     std::cout << std::to_string(byte) << " ";
    // }
    // std::cout << std::endl;
    
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<CanPubSub>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}
