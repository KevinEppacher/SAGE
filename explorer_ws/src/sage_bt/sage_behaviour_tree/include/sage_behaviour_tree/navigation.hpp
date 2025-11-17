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
         rclcpp::Node::SharedPtr node);

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

    // Phases: 1=left, 2=origin, 3=right
    int phase{0};
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


// -------------------- End of navigation.hpp -------------------- //

// class Spin : public BT::StatefulActionNode
// {
// public:
//     Spin(const std::string &name,
//          const BT::NodeConfiguration &config,
//          rclcpp::Node::SharedPtr node);

//     static BT::PortsList providedPorts();

//     BT::NodeStatus onStart() override;
//     BT::NodeStatus onRunning() override;
//     void onHalted() override;

// private:
//     // Internal helpers
//     double shortestReturn(double angle);
//     void cosineCallback(const std_msgs::msg::Float32::SharedPtr msg);
//     double findLikeliestYaw() const;
//     void publishCosineProfile();

//     // Marker publishing
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
//     std::string markerFrame{"map"};
//     int markerId{0};

//     // Yaw tracking (update per sample)
//     double currentYaw{0.0};

//     // Helper
//     void publishDirectionMarker(double yaw, float cosineVal);

//     // Core ROS & behaviour objects
//     rclcpp::Node::SharedPtr node;
//     std::shared_ptr<Robot> robot;
//     rclcpp_action::ResultCode navResult{};

//     // Spin configuration
//     double turnLeftAngle{0.0};
//     double turnRightAngle{0.0};
//     double spinDuration{15.0};
//     double spinTimeout{30.0};
//     double originYaw{0.0};   // yaw at onStart()
//     int phase{0};
//     bool done{false};
//     std::deque<double> sweepTargetsAbs;   // absolute yaws in 'map'
//     const double yaw_epsilon = 0.03;      // ~1.7 deg; treat as reached

//     // Time tracking
//     rclcpp::Clock steadyClock{RCL_STEADY_TIME};
//     rclcpp::Time startTimeSteady;

//     // Cosine similarity behaviour
//     bool returnToLikeliest{false};
//     std::string cosineTopic{"/value_map/cosine_similarity"};
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cosineSub;
//     std::vector<std::pair<double, float>> cosineSamples;
//     float lastCosine{0.0f};
// };
