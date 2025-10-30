#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include "multimodal_query_msgs/msg/semantic_prompt_array.hpp"
#include "evaluator_msgs/msg/evaluation_event.hpp"
#include "sage_behaviour_tree/robot.hpp"
#include <nav2_costmap_2d/cost_values.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/utils.h>
#include <fstream>

// ============================ KeepRunningUntilObjectFound ============================ //

class KeepRunningUntilObjectFound : public BT::DecoratorNode
{
    public:
        KeepRunningUntilObjectFound(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        // Called on every tick
        BT::NodeStatus tick() override;

        private:
        bool objectFound = false;
        bool anyExplorationNodes = true;
        bool initialized = false;  // for first-tick initialization
};

// ============================ ForEachEvaluationPrompt ============================ //

using evaluator_msgs::msg::EvaluationEvent;

class ForEachEvaluationPrompt : public BT::DecoratorNode
{
public:
    ForEachEvaluationPrompt(const std::string &name,
                            const BT::NodeConfig &config,
                            const rclcpp::Node::SharedPtr &nodePtr);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void callbackEvent(const EvaluationEvent::SharedPtr msg);
    // New: encapsulate lazy comms init
    void initCommsFromPorts();

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<EvaluationEvent>::SharedPtr subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher;

    bool subscribed{false};
    size_t currentIndex{0};
    std::vector<std::string> promptTexts;

    // Comms config/state
    bool commReady{false};
    std::string evaluation_event_topic_{"/evaluation/event"};
    std::string iteration_status_topic_{"/evaluation/iteration_status"};
};

// ============================ ApproachPoseAdjustor ============================ //

class ApproachPoseAdjustor : public BT::DecoratorNode
{
public:
    ApproachPoseAdjustor(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Robot> robot;

    // Search / validation parameters
    double approachRadius{2.5};                // meters (from input port)
    double robotRadius{0.25};                  // meters, robot circular footprint
    int costThreshold{50};                     // OccupancyGrid: >50 treated as obstacle
    bool allowUnknown{false};                  // treat -1 as obstacle if false
    double angularStepDeg{10.0};               // angle sweep step around target
    int radialSamples{3};                      // how many radii between 0..approachRadius
    double lineStep{0.05};                     // raycast discretization [m]
    bool haveCachedReachable{false};
    graph_node_msgs::msg::GraphNode cachedReachable;
    BT::NodeStatus lastChildStatus{BT::NodeStatus::IDLE};
    std::vector<geometry_msgs::msg::Point> lastSamples;
    graph_node_msgs::msg::GraphNode lastTarget;
    graph_node_msgs::msg::GraphNode lastReachable;
    bool lastReachableValid{false};

    // Visualization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub;
    std::string markerFrame{"map"};
    int markerSeq{0};

    // Core logic
    bool findReachablePoint(const graph_node_msgs::msg::GraphNode &target,
                            graph_node_msgs::msg::GraphNode &reachable);

    bool isFootprintFree(const nav_msgs::msg::OccupancyGrid &grid,
                         double x, double y,
                         double radius) const;

    bool rayPathAcceptable(const nav_msgs::msg::OccupancyGrid &grid,
                           double x0, double y0,
                           double x1, double y1) const;

    inline bool gridValueAcceptable(int8_t v) const
    {
        if (v < 0) return allowUnknown;     // -1 unknown
        return v <= costThreshold;          // 0..100
    }

    inline bool worldToMap(const nav_msgs::msg::OccupancyGrid &grid,
                           double wx, double wy,
                           int &mx, int &my) const
    {
        const auto &info = grid.info;
        mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
        my = static_cast<int>((wy - info.origin.position.y) / info.resolution);
        return (mx >= 0 && my >= 0 &&
                mx < static_cast<int>(info.width) &&
                my < static_cast<int>(info.height));
    }

    void publishMarkers(const geometry_msgs::msg::Pose &robotPose,
                        const graph_node_msgs::msg::GraphNode &target,
                        const graph_node_msgs::msg::GraphNode *reachable,
                        const std::string &ns);
};