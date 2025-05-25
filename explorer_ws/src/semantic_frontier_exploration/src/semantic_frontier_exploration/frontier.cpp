#include "frontier.hpp"

Frontier::Frontier(rclcpp::Node* node) : node_(node)
{
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_markers", 10);
    RCLCPP_INFO(node_->get_logger(), "Frontier helper initialized");

    
}

Frontier::~Frontier() {}