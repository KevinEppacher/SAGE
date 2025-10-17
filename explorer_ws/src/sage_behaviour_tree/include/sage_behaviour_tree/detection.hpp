#pragma once
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/condition_node.h"
#include <graph_node_msgs/msg/graph_node_array.hpp>
#include <graph_node_msgs/msg/graph_node.hpp>

class IsDetected : public BT::ConditionNode
{
public:
    IsDetected(const std::string& name,
               const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<graph_node_msgs::msg::GraphNodeArray>::SharedPtr sub_;
    graph_node_msgs::msg::GraphNodeArray::SharedPtr latest_msg_;
    bool received_message_{false};
    int missed_ticks_{8};
};
