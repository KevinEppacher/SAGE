#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/action_node.h"
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <graph_node_msgs/msg/graph_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

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
    GoToGraphNode(const std::string& name,
                  const BT::NodeConfiguration& config,
                  rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // --- Internal helpers ---
    void publishGoalToTarget(const graph_node_msgs::msg::GraphNode& node);
    bool targetChanged(const graph_node_msgs::msg::GraphNode& new_target) const;

    // --- ROS interfaces ---
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    // --- State ---
    std::shared_ptr<graph_node_msgs::msg::GraphNode> target_node_;
    rclcpp::Time last_publish_time_;
    std::string goal_topic_;
    double approach_radius_{2.0};
};