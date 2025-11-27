#ifndef OBSERVE_GRAPH_NODES_HPP
#define OBSERVE_GRAPH_NODES_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <graph_node_msgs/msg/graph_node_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <vector>
#include <string>
#include <limits>

// Custom
#include "sage_behaviour_tree/robot.hpp"

/// Utility: check if a cell is occupied in occupancy map
bool cellIsOccupied(const nav_msgs::msg::OccupancyGrid& map, int mx, int my);

// ============================================================
// GraphNodeManager
// ============================================================

class GraphNodeManager {
public:
    explicit GraphNodeManager(rclcpp::Node::SharedPtr node);

    bool noRecentGraph();
    std::shared_ptr<graph_node_msgs::msg::GraphNode> getBestScoreNode() const;
    bool allNodesObserved() const;

    graph_node_msgs::msg::GraphNodeArray::SharedPtr latestGraph;
    nav_msgs::msg::OccupancyGrid::SharedPtr latestMap;

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<graph_node_msgs::msg::GraphNodeArray>::SharedPtr subGraph;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subMap;

    bool receivedGraph = false;
    int missedTicks = 0;
    const int maxMissedTicks = 8;
};

// ============================================================
// MapRayTracer
// ============================================================

class MapRayTracer {
public:
    bool isVisible(const graph_node_msgs::msg::GraphNode& node,
                   const geometry_msgs::msg::Pose& robotPose,
                   const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                   double sightHorizon,
                   bool enableRaytrace) const;

private:
    bool performRaytrace(const nav_msgs::msg::OccupancyGrid& map,
                         const geometry_msgs::msg::Pose& start,
                         const geometry_msgs::msg::Point& end) const;
};

// ============================================================
// SpinController
// ============================================================

class SpinController {
public:
    enum class State { IDLE, SPINNING_MIN, SPINNING_HOME, SPINNING_MAX, DONE };

    explicit SpinController(std::unique_ptr<Robot>& robot);

    bool shouldTriggerSpin(const geometry_msgs::msg::Pose& pose, double threshold);
    void configure(double minYaw, double maxYaw);
    void startSpinToMin(const std::string& mapFrame);
    BT::NodeStatus updateSpinState(const std::string& mapFrame);
    void cancel();

    State getState() const { return state; }

private:
    std::unique_ptr<Robot>& robot;
    geometry_msgs::msg::Pose lastSpinPose;
    bool lastSpinPoseValid = false;
    double minYaw = 0.0, homeYaw = 0.0, maxYaw = 0.0;
    State state = State::IDLE;
};

// ============================================================
// VisibilityAnalyzer
// ============================================================

class VisibilityAnalyzer {
public:
    explicit VisibilityAnalyzer(const MapRayTracer& tracer);

    std::vector<const graph_node_msgs::msg::GraphNode*> findUnobservedVisible(
        const GraphNodeManager& manager,
        const geometry_msgs::msg::Pose& robotPose,
        double sightHorizon,
        bool performRaytrace) const;

    bool computeYawSpan(const std::vector<const graph_node_msgs::msg::GraphNode*>& visibleNodes,
                        const geometry_msgs::msg::Pose& robotPose,
                        double& outMinYaw,
                        double& outMaxYaw) const;

private:
    const MapRayTracer& rayTracer;
};

// ============================================================
// ObserveGraphNodes (BT Node)
// ============================================================

class ObserveGraphNodes : public BT::StatefulActionNode {
public:
    ObserveGraphNodes(const std::string& name,
                      const BT::NodeConfiguration& config,
                      rclcpp::Node::SharedPtr nodePtr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<Robot> robot;
    std::unique_ptr<GraphNodeManager> graphManager;
    std::unique_ptr<MapRayTracer> rayTracer;
    std::unique_ptr<SpinController> spinCtrl;
    std::unique_ptr<VisibilityAnalyzer> visibility;

    // Parameters
    bool performRayTracing = false;
    double sightHorizon = 10.0;
    double spinDistanceThreshold = 1.0;
    std::string graphNodeTopic = "/fused/exploration_graph_nodes/graph_nodes";
    std::string mapFrame = "map";
    std::string robotFrame = "base_link";
};

#endif  // OBSERVE_GRAPH_NODES_HPP
