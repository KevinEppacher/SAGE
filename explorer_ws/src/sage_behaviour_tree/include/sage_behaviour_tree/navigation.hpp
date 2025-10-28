#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/action_node.h"
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "sage_behaviour_tree/robot.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <graph_node_msgs/msg/graph_node.hpp>
#include <graph_node_msgs/msg/graph_node_array.hpp>

// =============================== Spin =================================== //

class Spin : public BT::StatefulActionNode
{
public:
    Spin(const std::string &name,
         const BT::NodeConfiguration &config,
         rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double shortestReturn(double angle);

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Robot> robot;

    bool done{false};
    int phase{0};
    double turnLeftAngle{0.0};
    double turnRightAngle{0.0};
    double spinDuration{15.0};
    double cumulativeRotation{0.0};
    rclcpp_action::ResultCode navResult{};
};

// ============================ GoToGraphNode ============================ //

class GoToGraphNode : public BT::StatefulActionNode
{
public:
    GoToGraphNode(const std::string &name,
                  const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool isWithinGoal(const graph_node_msgs::msg::GraphNode &node);

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Robot> robot;
    std::shared_ptr<graph_node_msgs::msg::GraphNode> target;

    std::string mapFrame{"map"};
    std::string robotFrame{"base_link"};
    std::string goalTopic{"/goal_pose"};

    double approachRadius{2.0};
    double timeoutSec{60.0};
    rclcpp::Time lastPublishTime;
    rclcpp::Time startTime;
};

// ============================ RealignToObject ============================ //

class RealignToObject : public BT::StatefulActionNode
{
public:
    RealignToObject(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double computeYawToTarget(const geometry_msgs::msg::Pose &robotPose,
                              const graph_node_msgs::msg::GraphNode &target);

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Robot> robot;

    double targetYaw{0.0};
    double spinDuration{5.0};
    bool started{false};
};
