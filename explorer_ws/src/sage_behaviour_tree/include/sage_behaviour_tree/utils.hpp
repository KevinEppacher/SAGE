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
// Custom Class
#include "sage_behaviour_tree/robot.hpp"

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

class SeekoutGraphNodes : public BT::SyncActionNode
{
public:
    SeekoutGraphNodes(const std::string& name,
                      const BT::NodeConfiguration& config,
                      rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void initSubscription(const std::string& topic);
    bool computeYawRange(const geometry_msgs::msg::Pose& robot_pose,
                         double sight_horizon,
                         double& min_yaw,
                         double& max_yaw);
    void publishMarker(const geometry_msgs::msg::Pose& robot_pose,
                       double sight_horizon,
                       double min_yaw,
                       double max_yaw);

    rclcpp::Node::SharedPtr node_ptr_;
    std::unique_ptr<Robot> robot_;

    rclcpp::Subscription<graph_node_msgs::msg::GraphNodeArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    graph_node_msgs::msg::GraphNodeArray::SharedPtr latest_msg_;

    bool received_message_{false};
    int missed_ticks_{0};

    static constexpr int MAX_MISSED_TICKS = 10;
};