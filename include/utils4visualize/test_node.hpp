#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

#include "utils4visualize/visualizer.hpp"

namespace utils4visualize {

class TestNode : public rclcpp::Node {
public:
    TestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void cb_pose_with_cov(geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr pose);
    void cb_pose_with_cov_stamped(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose);
    void cb_odom(nav_msgs::msg::Odometry::ConstSharedPtr odom);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr sub_pose_with_cov_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_stamped_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    std::string pose_with_cov_topic_;
    std::string pose_with_cov_stamped_topic_;
    std::string odom_topic_;

    Visualizer visualizer_;
};

} // namespace utils4visualize
