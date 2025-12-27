#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include "multimodal_query_msgs/msg/semantic_prompt_array.hpp"
#include "sage_behaviour_tree/robot.hpp"
#include <nav2_costmap_2d/cost_values.hpp>
#include <sage_bt_msgs/srv/startup_check.hpp>
#include <sage_bt_msgs/action/execute_prompt.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.h>
#include <fstream>

using ExecutePrompt = sage_bt_msgs::action::ExecutePrompt;

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

// ============================ ApproachPoseAdjustor ============================ //

class ApproachPoseAdjustor : public BT::DecoratorNode
{
public:
    ApproachPoseAdjustor(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node,
                         std::shared_ptr<Robot> robotPtr);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Robot> robot;

    // Search / validation parameters
    double searchRadius{2.5};                // meters (from input port)
    double robotRadius{0.25};                  // meters, robot circular footprint
    int costThreshold{50};                     // OccupancyGrid: >50 treated as obstacle
    bool allowUnknown{false};                  // treat -1 as obstacle if false
    double angularStepDeg{10.0};               // angle sweep step around target
    int radialSamples{3};                      // how many radii between 0..searchRadius
    double lineStep{0.05};                     // raycast discretization [m]
    bool haveCachedReachable{false};
    bool debugMarkers{false};
    graph_node_msgs::msg::GraphNode cachedReachable;
    std::optional<graph_node_msgs::msg::GraphNode> lastTargetNode;
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

// ============================ SageBtOrchestrator ============================ //

class SageBtOrchestrator : public BT::DecoratorNode
{
public:
    using ExecutePrompt = sage_bt_msgs::action::ExecutePrompt;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ExecutePrompt>;

    SageBtOrchestrator(
        const std::string &name,
        const BT::NodeConfig &config,
        const rclcpp::Node::SharedPtr &node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void declareIfNotDeclared(const std::string &param_name,
                              const rclcpp::ParameterValue &default_value);
    void loadParameters();

    // StartupCheck service
    void onStartupRequest(
        const std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Request>,
        std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Response> resp);

    // Action server callbacks
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExecutePrompt::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
    void executeGoal(const std::shared_ptr<GoalHandle> goal_handle);

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr forceExitService;
    std::atomic<bool> forceExitRequested = false;

    void onForceExit(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>);

private:
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<Robot> robot;

    rclcpp::Service<sage_bt_msgs::srv::StartupCheck>::SharedPtr startupService;
    rclcpp_action::Server<ExecutePrompt>::SharedPtr actionServer;

    bool startupDone = false;
    bool activeGoal = false;

    std::vector<std::string> required_topics;
    std::vector<std::string> required_services;

    std::string currentPrompt;
    std::string currentSavePath;
};