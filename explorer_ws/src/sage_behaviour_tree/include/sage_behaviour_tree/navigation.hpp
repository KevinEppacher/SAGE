#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/action_node.h"
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <graph_node_msgs/msg/graph_node.hpp>

class Spin360 : public BT::StatefulActionNode
{
public:
    using Spin = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

    Spin360(const std::string& name,
            const BT::NodeConfiguration& config,
            rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void resultCallback(const GoalHandleSpin::WrappedResult& result);

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<Spin>::SharedPtr client_ptr_;
    std::shared_future<typename GoalHandleSpin::SharedPtr> goal_handle_future_;
    bool done_flag_{false};
    rclcpp_action::ResultCode nav_result_{};
};


// ============================ GoToGraphNode ============================ //

class GoToGraphNode : public BT::StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoToGraphNode(const std::string& name,
                  const BT::NodeConfiguration& config,
                  rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void resultCallback(const GoalHandleNav::WrappedResult& result);

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    std::shared_ptr<graph_node_msgs::msg::GraphNode> target_node_;
    bool done_flag_{false};
    rclcpp_action::ResultCode nav_result_{};
    std::shared_future<typename GoalHandleNav::SharedPtr> goal_handle_future_;
};
