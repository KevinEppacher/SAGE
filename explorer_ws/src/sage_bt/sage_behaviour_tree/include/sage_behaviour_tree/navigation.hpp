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
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>

// =============================== Spin =================================== //

class Spin : public BT::StatefulActionNode
{
public:
    Spin(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr nodePtr,
            std::shared_ptr<Robot> robotPtr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // ROS
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Robot> robot;

    // Inputs
    double turnLeftAngle{0.0};
    double turnRightAngle{0.0};
    double spinDuration{10.0};

    // Internal yaw state
    double originYaw{0.0};
    double leftTarget{0.0};
    double rightTarget{0.0};

    // Timing
    rclcpp::Clock steadyClock{RCL_STEADY_TIME};
    rclcpp::Time startTimeSteady;
    double spinTimeout{30.0};

    // Parameter and tracking
    double spinDistanceThreshold{1.0};  // [m]
    std::optional<geometry_msgs::msg::Point> lastSpinPosition;

    // Phase tracking
    int phase{0};
    int skippedSpins{0};

    // Helper
    bool isNearLastSpin(const geometry_msgs::msg::Point &pos) const;
};


// ============================ GoToGraphNode ============================ //

class GoToGraphNode : public BT::StatefulActionNode
    {
    public:
    GoToGraphNode(const std::string &name,
                                const BT::NodeConfiguration &config,
                                rclcpp::Node::SharedPtr nodePtr,
                                std::shared_ptr<Robot> robotPtr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool targetChanged(const graph_node_msgs::msg::GraphNode &p1,
                        const graph_node_msgs::msg::GraphNode &p2,
                        double threshold) const;
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Robot> robot;

    std::shared_ptr<graph_node_msgs::msg::GraphNode> target;
    
    // Parameters
    std::string mapFrame{"map"};
    std::string robotFrame{"base_link"};
    double approachRadius{0.5};
    double timeoutSec{120.0};

    rclcpp::Time startTime;
    bool goalSent = false;
    bool goalDone = false;
    bool goalSucceeded = false;

};


// ============================ RealignToObject ============================ //

class RealignToObject : public BT::StatefulActionNode
{
public:
    RealignToObject(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr nodePtr,
                    std::shared_ptr<Robot> robotPtr);

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


// -------------------- End of navigation.hpp -------------------- //