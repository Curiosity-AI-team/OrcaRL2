#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gazebo_ros/node.hpp>

using namespace gazebo;

class PosePublisher : public ModelPlugin
{
private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
        this->model = _model;

        // Initialize ROS 2 node
        this->node = rclcpp::Node::make_shared("pose_publisher_node");

        // Setup ROS 2 publisher
        this->publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("ball_pose", 10);

        // Connect to the world update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&PosePublisher::OnUpdate, this));
    }

    void OnUpdate()
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->node->get_clock()->now();
        msg.header.frame_id = "world";
        msg.pose.position.x = this->model->WorldPose().Pos().X();
        msg.pose.position.y = this->model->WorldPose().Pos().Y();
        msg.pose.position.z = this->model->WorldPose().Pos().Z();
        // msg.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(this->model->WorldPose().Rot());

        this->publisher->publish(msg);
    }
};

GZ_REGISTER_MODEL_PLUGIN(PosePublisher)
