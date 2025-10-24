#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <graph_node_msgs/msg/graph_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>
#include <chrono>
#include <graph_node_msgs/msg/graph_node_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/empty.hpp>
// Custom Class
#include "sage_behaviour_tree/robot.hpp"

// -------------------- SetParameterNode -------------------- //

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

// -------------------- SeekoutGraphNodes -------------------- //

class SeekoutGraphNodes : public BT::StatefulActionNode
{
public:
  SeekoutGraphNodes(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr nodePtr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void initSubscription(const std::string &topic);
  bool computeYawRange(const geometry_msgs::msg::Pose &robotPose,
                       double sightHorizon,
                       double &minYaw,
                       double &maxYaw);

  rclcpp::Node::SharedPtr nodePtr_;
  std::unique_ptr<Robot> robot_;

  rclcpp::Subscription<graph_node_msgs::msg::GraphNodeArray>::SharedPtr sub_;
  graph_node_msgs::msg::GraphNodeArray::SharedPtr latestMsg_;
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};

  bool receivedMsg_{false};
  rclcpp::Time startTime_;
  double timeoutSec_{30.0};

  // cached inputs
  std::string topic_;
  std::string mapFrame_;
  std::string robotFrame_;
  double horizon_{10.0};
  double minDef_{-M_PI};
  double maxDef_{M_PI};

};

// -------------------- CallEmptyService -------------------- //

class CallEmptyService : public BT::StatefulActionNode
{
public:
    CallEmptyService(const std::string& name,
                     const BT::NodeConfiguration& config,
                     rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client;
    std::string serviceName;

    std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> future;

    // Clock tracking to avoid mixed clock subtraction errors
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Time lastTick;
    double accumulatedSeconds{0.0};
};
