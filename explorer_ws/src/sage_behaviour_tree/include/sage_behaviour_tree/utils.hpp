#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <chrono>

class SetParameterNode : public BT::SyncActionNode
{
public:
    SetParameterNode(const std::string& name,
                     const BT::NodeConfiguration& config,
                     rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
};