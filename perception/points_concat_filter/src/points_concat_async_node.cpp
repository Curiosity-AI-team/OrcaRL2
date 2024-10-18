#include "points_concat_async_node.h"

namespace perception {

PointsConcatAsyncFilterNode::PointsConcatAsyncFilterNode(const std::string &node_name)
        : Node(node_name),
            timeout_(rclcpp::Duration::from_seconds(1.0)),
            b_front_(false),
            b_left_(false),
            b_right_(false),
            b_transform_list_generated_(false) {

    RCLCPP_INFO(this->get_logger(), "Initializing PointsConcatAsyncFilterNode");

    // Initialize subscribers
    sub_points_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/front_points", rclcpp::SensorDataQoS(),
            std::bind(&PointsConcatAsyncFilterNode::frontCallback, this, std::placeholders::_1));

    sub_points_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/left_points", rclcpp::SensorDataQoS(),
            std::bind(&PointsConcatAsyncFilterNode::leftCallback, this, std::placeholders::_1));

    sub_points_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/right_points", rclcpp::SensorDataQoS(),
            std::bind(&PointsConcatAsyncFilterNode::rightCallback, this, std::placeholders::_1));

    // Initialize publisher
//     pub_concat_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             "/concat/points2", rclcpp::SensorDataQoS());

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    pub_concat_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/concat/points2", qos_profile);

    // Initialize timer
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PointsConcatAsyncFilterNode::timerCallback, this));

    // Generate transformation list
    makeTransformList();
}

PointsConcatAsyncFilterNode::~PointsConcatAsyncFilterNode() {}

void PointsConcatAsyncFilterNode::frontCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    front_last_update_ = this->now();
    b_front_ = true;
    
    // Create a temporary PointCloud to check the type
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Try to convert the message to a PointCloud<PointXYZ>
    try {
        pcl::fromROSMsg(*msg, *temp_xyz_cloud);    // If this succeeds, we have PointXYZ data
        // Convert PointXYZ to PointXYZI
        m_front_points_ = convertToXYZI(temp_xyz_cloud, 1.0f);    // Default intensity value
    } 
    catch (const pcl::PCLException& e) {
        // If conversion to PointXYZ fails, assume it's PointXYZI
        RCLCPP_INFO(this->get_logger(), "Converting directly to PointXYZI.");
        m_front_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *m_front_points_);    // Directly convert to PointXYZI
    }
}

void PointsConcatAsyncFilterNode::leftCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    left_last_update_ = this->now();
    b_left_ = true;
    m_left_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *m_left_points_);
}

void PointsConcatAsyncFilterNode::rightCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    right_last_update_ = this->now();
    b_right_ = true;
    m_right_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *m_right_points_);
}

void PointsConcatAsyncFilterNode::timerCallback() {
    if (!b_transform_list_generated_) {
        return;
    }

    auto concatenated_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Front Lidar
    if (b_front_ && (this->now() - front_last_update_) <= timeout_) {
        auto transformed_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        transformPointCloudCustom(*m_front_points_, *transformed_points, transform_list_[0]);
        *concatenated_points += *transformed_points;
    }

    // Left Lidar
    if (b_left_ && (this->now() - left_last_update_) <= timeout_) {
        auto transformed_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        transformPointCloudCustom(*m_left_points_, *transformed_points, transform_list_[1]);
        *concatenated_points += *transformed_points;
    }

    // Right Lidar
    if (b_right_ && (this->now() - right_last_update_) <= timeout_) {
        auto transformed_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        transformPointCloudCustom(*m_right_points_, *transformed_points, transform_list_[2]);
        *concatenated_points += *transformed_points;
    }

    // Publish concatenated point cloud
    if (!concatenated_points->empty()) {
        sensor_msgs::msg::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(*concatenated_points, merged_cloud_msg);
        merged_cloud_msg.header.frame_id = "base_link";
        merged_cloud_msg.header.stamp = this->now();
        pub_concat_points_->publish(merged_cloud_msg);
    }
}

void PointsConcatAsyncFilterNode::makeTransformList() {
    // Transformation to the center of gravity
    tf2::Transform transform_to_cog;
    transform_to_cog.setOrigin(tf2::Vector3(-1.3206, 0.030188, -0.23598));
    tf2::Quaternion quat_to_cog;
    quat_to_cog.setRPY(0.0, 0.0, 0.0);
    transform_to_cog.setRotation(quat_to_cog);

    // Lidar rotation (identity in this case)
    tf2::Transform transform_lidar_rotate;
    transform_lidar_rotate.setIdentity();

    // Front Lidar
    tf2::Transform transform_front;
    transform_front.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion quat_front;
    quat_front.setRPY(0.0, 0.0, 0.0);
    transform_front.setRotation(quat_front);
    transform_list_.push_back(transform_to_cog * transform_front * transform_lidar_rotate);

    // Left Lidar
    tf2::Transform transform_left;
    transform_left.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion quat_left;
//     quat_left.setRPY(0.0, 0.0, 2.0943951024); // 120 degrees
    quat_left.setRPY(0.0, 0.0, 0.0);
    transform_left.setRotation(quat_left);
    transform_list_.push_back(transform_to_cog * transform_left * transform_lidar_rotate);

    // Right Lidar
    tf2::Transform transform_right;
    transform_right.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion quat_right;
//     quat_right.setRPY(0.0, 0.0, -2.0943951024); // -120 degrees
    quat_left.setRPY(0.0, 0.0, 0.0);
    transform_right.setRotation(quat_right);
    transform_list_.push_back(transform_to_cog * transform_right * transform_lidar_rotate);

    b_transform_list_generated_ = true;
}

void PointsConcatAsyncFilterNode::transformPointCloudCustom(
        const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
        pcl::PointCloud<pcl::PointXYZI> &cloud_out,
        const tf2::Transform &transform) {

    Eigen::Matrix4f eigen_transform;
    tf2::Matrix3x3 rotation = transform.getBasis();
    tf2::Vector3 origin = transform.getOrigin();

    eigen_transform << rotation[0][0], rotation[0][1], rotation[0][2], origin.x(),
                                         rotation[1][0], rotation[1][1], rotation[1][2], origin.y(),
                                         rotation[2][0], rotation[2][1], rotation[2][2], origin.z(),
                                         0, 0, 0, 1;

    pcl::transformPointCloud(cloud_in, cloud_out, eigen_transform);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointsConcatAsyncFilterNode::downsample(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution) {
    auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*filtered);
    return filtered;
}

} // namespace perception