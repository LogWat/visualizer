#include "utils4visualize/test_node.hpp"

namespace utils4visualize {

TestNode::TestNode(const rclcpp::NodeOptions & options)
    : Node("test_node", options), visualizer_() {
    this->declare_parameter<std::string>("pose_with_cov_topic", "dummy_input");
    this->declare_parameter<std::string>("pose_with_cov_stamped_topic", "/ndt_pose_with_covariance");
    this->declare_parameter<std::string>("odom_topic", "/ekf_odom");
    this->declare_parameter<std::string>("twist_with_cov_stamped_topic", "/gyro_twist_with_covariance");
    this->get_parameter("pose_with_cov_topic", pose_with_cov_topic_);
    this->get_parameter("pose_with_cov_stamped_topic", pose_with_cov_stamped_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("twist_with_cov_stamped_topic", twist_with_cov_stamped_topic_);

    sub_pose_with_cov_ = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(
        pose_with_cov_topic_,
        10,
        std::bind(&TestNode::cb_pose_with_cov, this, std::placeholders::_1)
    );
    sub_pose_with_cov_stamped_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_with_cov_stamped_topic_,
        10,
        std::bind(&TestNode::cb_pose_with_cov_stamped, this, std::placeholders::_1)
    );
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        10,
        std::bind(&TestNode::cb_odom, this, std::placeholders::_1)
    );
    sub_twist_with_cov_stamped_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_with_cov_stamped_topic_,
        10,
        std::bind(&TestNode::cb_twist_with_cov_stamped, this, std::placeholders::_1)
    );
}

void TestNode::cb_pose_with_cov(geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr pose) {
    RCLCPP_INFO(this->get_logger(), "pose_with_cov: x=%f, y=%f, z=%f", pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);

}

void TestNode::cb_pose_with_cov_stamped(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose) {
    
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    covariance(0, 0) = pose->pose.covariance[0];
    covariance(0, 1) = pose->pose.covariance[1];
    covariance(1, 0) = pose->pose.covariance[6];
    covariance(1, 1) = pose->pose.covariance[7];
    Eigen::Vector2f mean;
    mean << pose->pose.pose.position.x, pose->pose.pose.position.y;
    RCLCPP_INFO(this->get_logger(), "NDT Covariance: %f, %f, %f, %f", covariance(0, 0), covariance(0, 1), covariance(1, 0), covariance(1, 1));
    visualizer_.ndt_covariance_.reset(new Eigen::Matrix2f(covariance));
    visualizer_.ndt_mean_.reset(new Eigen::Vector2f(mean));
}

void TestNode::cb_odom(nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    // RCLCPP_INFO(this->get_logger(), "odom: x=%f, y=%f, z=%f", odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    covariance(0, 0) = odom->pose.covariance[0];
    covariance(0, 1) = odom->pose.covariance[1];
    covariance(1, 0) = odom->pose.covariance[6];
    covariance(1, 1) = odom->pose.covariance[7];
    Eigen::Vector2f mean;
    mean << odom->pose.pose.position.x, odom->pose.pose.position.y;
    RCLCPP_INFO(this->get_logger(), "EKF Covariance: %f, %f, %f, %f", covariance(0, 0), covariance(0, 1), covariance(1, 0), covariance(1, 1));
    visualizer_.plot_covariance_ellipse_detail(covariance, mean);
}

void TestNode::cb_twist_with_cov_stamped(geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist) {
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    covariance(0, 0) = twist->twist.covariance[0];
    covariance(0, 1) = twist->twist.covariance[1];
    covariance(1, 0) = twist->twist.covariance[6];
    covariance(1, 1) = twist->twist.covariance[7];
    Eigen::Vector2f mean;
    mean << twist->twist.twist.linear.x, twist->twist.twist.linear.y;
    RCLCPP_INFO(this->get_logger(), "Gyro Covariance: %f, %f, %f, %f", covariance(0, 0), covariance(0, 1), covariance(1, 0), covariance(1, 1));
    visualizer_.twist_covariance_.reset(new Eigen::Matrix2f(covariance));
    visualizer_.twist_mean_.reset(new Eigen::Vector2f(mean));
}

} // namespace utils4visualize

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<utils4visualize::TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
