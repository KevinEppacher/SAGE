#pragma once
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/condition_node.h"
#include <graph_node_msgs/msg/graph_node_array.hpp>
#include <graph_node_msgs/msg/graph_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/qos.hpp>

// ============================ IsDetected ============================ //

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

// ============================ SaveImageAction ============================ //

class SaveImageAction : public BT::StatefulActionNode
{
public:
    SaveImageAction(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

    std::string topic;
    std::string savePath;
    bool done{false};
    rclcpp::Time startTime;
};
