#include "sage_behaviour_tree/observe_graph_nodes.hpp"
#include "sage_behaviour_tree/colors.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// ============================================================
// Utility
// ============================================================

bool cellIsOccupied(const nav_msgs::msg::OccupancyGrid& map, int mx, int my) {
    if (mx < 0 || my < 0) return false;
    if (mx >= static_cast<int>(map.info.width)) return false;
    if (my >= static_cast<int>(map.info.height)) return false;

    int idx = my * map.info.width + mx;
    int8_t value = map.data[idx];
    return value >= 50;
}

// ============================================================
// GraphNodeManager
// ============================================================

GraphNodeManager::GraphNodeManager(rclcpp::Node::SharedPtr nodePtr)
    : node(std::move(nodePtr))
{
    subGraph = node->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        "/fused/exploration_graph_nodes/graph_nodes", 10,
        [this](graph_node_msgs::msg::GraphNodeArray::SharedPtr msg) {
            latestGraph = std::move(msg);
            receivedGraph = true;
            missedTicks = 0;
            // RCLCPP_INFO(node->get_logger(),
            //             GREEN "[GraphNodeManager] Received %zu graph nodes." RESET,
            //             latestGraph->nodes.size());
        });

    subMap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latestMap = std::move(msg);
            // RCLCPP_INFO(node->get_logger(), GREEN "[GraphNodeManager] Received updated occupancy map." RESET);
        });
}

bool GraphNodeManager::noRecentGraph() {
    if (!receivedGraph) {
        missedTicks++;
        RCLCPP_WARN(node->get_logger(),
                    ORANGE "[GraphNodeManager] Waiting for graph nodes... (%d/%d)" RESET,
                    missedTicks, maxMissedTicks);

        if (missedTicks > maxMissedTicks) {
            RCLCPP_ERROR(node->get_logger(),
                         RED "[GraphNodeManager] No graph nodes received for %d ticks → FAILURE" RESET,
                         missedTicks);
            return true;
        }
        return true; // still waiting
    }
    receivedGraph = false;
    missedTicks = 0;
    return false;
}

std::shared_ptr<graph_node_msgs::msg::GraphNode> GraphNodeManager::getBestScoreNode() const {
    if (!latestGraph || latestGraph->nodes.empty()) return nullptr;

    const graph_node_msgs::msg::GraphNode* bestNode = nullptr;
    double bestScore = -std::numeric_limits<double>::infinity();

    for (const auto& n : latestGraph->nodes) {
        if (n.score > bestScore) {
            bestScore = n.score;
            bestNode = &n;
        }
    }

    if (!bestNode) return nullptr;

    // RCLCPP_INFO(node->get_logger(),
    //             GREEN "[GraphNodeManager] Best node → ID: %d | Score: %.3f" RESET,
    //             bestNode->id, bestNode->score);

    return std::make_shared<graph_node_msgs::msg::GraphNode>(*bestNode);
}

bool GraphNodeManager::allNodesObserved() const {
    if (!latestGraph) return false;
    for (const auto& n : latestGraph->nodes) {
        if (!n.is_observed) return false;
    }
    RCLCPP_INFO(node->get_logger(), GREEN "[GraphNodeManager] All graph nodes observed." RESET);
    return true;
}

void GraphNodeManager::shutdown() {
    if (subGraph) {
        subGraph.reset();
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[GraphNodeManager] Graph subscription reset." RESET);
    }
    if (subMap) {
        subMap.reset();
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[GraphNodeManager] Map subscription reset." RESET);
    }
    latestGraph.reset();
    latestMap.reset();
}


// ============================================================
// MapRayTracer
// ============================================================

bool MapRayTracer::isVisible(const graph_node_msgs::msg::GraphNode& node,
                             const geometry_msgs::msg::Pose& robotPose,
                             const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                             double sightHorizon,
                             bool enableRaytrace) const
{
    double dx = node.position.x - robotPose.position.x;
    double dy = node.position.y - robotPose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist > sightHorizon)
        return false;

    if (!enableRaytrace || !map)
        return true;

    return performRaytrace(*map, robotPose, node.position);
}

bool MapRayTracer::performRaytrace(const nav_msgs::msg::OccupancyGrid& map,
                                   const geometry_msgs::msg::Pose& start,
                                   const geometry_msgs::msg::Point& end) const
{
    auto worldToMap = [&](double wx, double wy, int& mx, int& my) {
        mx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
        my = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
    };

    int rx, ry, tx, ty;
    worldToMap(start.position.x, start.position.y, rx, ry);
    worldToMap(end.x, end.y, tx, ty);

    if (rx < 0 || ry < 0 || tx < 0 || ty < 0 ||
        rx >= (int)map.info.width || ry >= (int)map.info.height ||
        tx >= (int)map.info.width || ty >= (int)map.info.height)
    {
        return true;
    }

    int x0 = rx, y0 = ry, x1 = tx, y1 = ty;
    int dx_i = std::abs(x1 - x0), dy_i = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx_i - dy_i;

    while (true) {
        if (!(x0 == rx && y0 == ry)) {
            if (cellIsOccupied(map, x0, y0)) return false;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy_i) { err -= dy_i; x0 += sx; }
        if (e2 < dx_i) { err += dx_i; y0 += sy; }
    }
    return true;
}

// ============================================================
// SpinController
// ============================================================

SpinController::SpinController(std::unique_ptr<Robot>& robotRef,
                               rclcpp::Node::SharedPtr nodePtr)
    : robot(robotRef), node(std::move(nodePtr))
{}

bool SpinController::shouldTriggerSpin(const geometry_msgs::msg::Pose& pose, double threshold) {
    if (!lastSpinPoseValid) {
        lastSpinPose = pose;
        lastSpinPoseValid = true;
        return true;
    }

    double dx = pose.position.x - lastSpinPose.position.x;
    double dy = pose.position.y - lastSpinPose.position.y;

    if (std::hypot(dx, dy) > threshold) {
        lastSpinPose = pose;
        return true;
    }
    return false;
}

void SpinController::configure(double minYawVal, double maxYawVal) {
    minYaw = minYawVal;
    maxYaw = maxYawVal;
    homeYaw = (minYaw + maxYaw) / 2.0;
}

void SpinController::startSpinToMin(const std::string& mapFrame) {
    robot->spinRelativeTo(mapFrame, minYaw, 4.0);
    state = State::SPINNING_MIN;
    RCLCPP_INFO(node->get_logger(),
                ORANGE "[SpinController] Starting spin to MIN yaw (%.2f°)" RESET,
                minYaw * 180.0 / M_PI);
}

BT::NodeStatus SpinController::updateSpinState(const std::string& mapFrame) {
    switch (state) {
        case State::SPINNING_MIN:
            if (robot->isSpinDone()) {
                RCLCPP_INFO(node->get_logger(),
                            GREEN "[SpinController] Spin MIN complete → returning HOME" RESET);
                robot->spinRelativeTo(mapFrame, homeYaw, 4.0);
                state = State::SPINNING_HOME;
            }
            break;

        case State::SPINNING_HOME:
            if (robot->isSpinDone()) {
                RCLCPP_INFO(node->get_logger(),
                            GREEN "[SpinController] Returned HOME → spinning MAX" RESET);
                robot->spinRelativeTo(mapFrame, maxYaw, 4.0);
                state = State::SPINNING_MAX;
            }
            break;

        case State::SPINNING_MAX:
            if (robot->isSpinDone()) {
                RCLCPP_INFO(node->get_logger(),
                            GREEN "[SpinController] Spin MAX complete → DONE" RESET);
                state = State::DONE;
                return BT::NodeStatus::SUCCESS;
            }
            break;

        default:
            break;
    }
    return BT::NodeStatus::RUNNING;
}

void SpinController::cancel() {
    robot->cancelSpin();
    state = State::IDLE;
    RCLCPP_WARN(node->get_logger(),
                YELLOW "[SpinController] Spin canceled by BT halt." RESET);
}

// ============================================================
// VisibilityAnalyzer
// ============================================================

VisibilityAnalyzer::VisibilityAnalyzer(const MapRayTracer& tracer)
    : rayTracer(tracer)
{}

std::vector<const graph_node_msgs::msg::GraphNode*> VisibilityAnalyzer::findUnobservedVisible(
    const GraphNodeManager& manager,
    const geometry_msgs::msg::Pose& robotPose,
    double sightHorizon,
    bool performRaytrace) const
{
    std::vector<const graph_node_msgs::msg::GraphNode*> result;
    if (!manager.latestGraph) return result;

    int unobservedCount = 0;
    for (const auto& n : manager.latestGraph->nodes) {
        if (!n.is_observed) {
            unobservedCount++;
            if (rayTracer.isVisible(n, robotPose, manager.latestMap, sightHorizon, performRaytrace))
                result.push_back(&n);
        }
    }

    // RCLCPP_INFO(manager.getLogger(),
    //             ORANGE "[VisibilityAnalyzer] %d unobserved nodes, %zu visible." RESET,
    //             unobservedCount, result.size());
    return result;
}

bool VisibilityAnalyzer::computeYawSpan(
    const std::vector<const graph_node_msgs::msg::GraphNode*>& visibleNodes,
    const geometry_msgs::msg::Pose& robotPose,
    double& outMinYaw,
    double& outMaxYaw) const
{
    if (visibleNodes.empty()) return false;

    std::vector<double> yaws;
    yaws.reserve(visibleNodes.size());
    for (auto n : visibleNodes) {
        double dx = n->position.x - robotPose.position.x;
        double dy = n->position.y - robotPose.position.y;
        yaws.push_back(std::atan2(dy, dx));
    }

    auto [minIt, maxIt] = std::minmax_element(yaws.begin(), yaws.end());
    outMinYaw = *minIt;
    outMaxYaw = *maxIt;

    RCLCPP_INFO(rclcpp::get_logger("VisibilityAnalyzer"),
                ORANGE "[VisibilityAnalyzer] Yaw span [%.1f°, %.1f°]" RESET,
                outMinYaw * 180.0 / M_PI, outMaxYaw * 180.0 / M_PI);
    return true;
}

// ============================================================
// ObserveGraphNodes (BT Node)
// ============================================================

ObserveGraphNodes::ObserveGraphNodes(const std::string& name,
                                     const BT::NodeConfiguration& config,
                                     rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_unique<Robot>(node))
{
    graphManager = std::make_unique<GraphNodeManager>(node);
    rayTracer = std::make_unique<MapRayTracer>();
    spinCtrl = std::make_unique<SpinController>(robot, node);
    visibility = std::make_unique<VisibilityAnalyzer>(*rayTracer);

    // If not declred, declare
    if(!node->has_parameter("observe.perform_raytracing"))
        node->declare_parameter("observe.perform_raytracing", false);
    if(!node->has_parameter("observe.sight_horizon"))
        node->declare_parameter("observe.sight_horizon", 10.0);
    if(!node->has_parameter("observe.spin_distance_threshold"))
        node->declare_parameter("observe.spin_distance_threshold", 1.0);
    if(!node->has_parameter("observe.map_frame"))
        node->declare_parameter("observe.map_frame", "map");
    if(!node->has_parameter("observe.robot_frame"))
        node->declare_parameter("observe.robot_frame", "base_link");
    if(!node->has_parameter("observe.timeout_sec"))
        node->declare_parameter("observe.timeout_sec", 90.0);
    if(!node->has_parameter("observe.value_map_node"))
        node->declare_parameter("observe.value_map_node", "/value_map/value_map");
    if(!node->has_parameter("observe.decay_factor_param"))
        node->declare_parameter("observe.decay_factor_param", "semantic_map.decay_factor");
    if(!node->has_parameter("observe.decay_factor_default"))
        node->declare_parameter("observe.decay_factor_default", 0.9995);

    performRayTracing = node->get_parameter("observe.perform_raytracing").as_bool();
    sightHorizon = node->get_parameter("observe.sight_horizon").as_double();
    spinDistanceThreshold = node->get_parameter("observe.spin_distance_threshold").as_double();
    mapFrame = node->get_parameter("observe.map_frame").as_string();
    robotFrame = node->get_parameter("observe.robot_frame").as_string();
    timeoutSec = node->get_parameter("observe.timeout_sec").as_double();
    valueMapNode = node->get_parameter("observe.value_map_node").as_string();
    decayFactorParam = node->get_parameter("observe.decay_factor_param").as_string();
    valueMapDecayFactorDefault = static_cast<float>(
        node->get_parameter("observe.decay_factor_default").as_double());

    RCLCPP_INFO(node->get_logger(),
                GREEN "[ObserveGraphNodes] Configured timeout: %.1f s" RESET,
                timeoutSec);

    RCLCPP_INFO(node->get_logger(), GREEN "[ObserveGraphNodes] Node initialized successfully." RESET);
}

BT::PortsList ObserveGraphNodes::providedPorts() {
    return {
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("likiest_explorer_node"),
        BT::OutputPort<bool>("any_exploration_nodes")
    };
}

BT::NodeStatus ObserveGraphNodes::onStart() {
    RCLCPP_INFO(node->get_logger(),
                ORANGE "[ObserveGraphNodes] Starting observation sequence..." RESET);

    startTime = steadyClock.now();
    timerActive = true;

    if (graphManager->noRecentGraph())
        return BT::NodeStatus::RUNNING;

    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose, mapFrame, robotFrame)) {
        RCLCPP_WARN(node->get_logger(),
                    ORANGE "[ObserveGraphNodes] Waiting for valid robot pose..." RESET);
        return BT::NodeStatus::RUNNING;
    }

    auto bestNode = graphManager->getBestScoreNode();
    if (bestNode)
        setOutput("likiest_explorer_node", bestNode);

    auto visibleNodes = visibility->findUnobservedVisible(*graphManager, robotPose, sightHorizon, performRayTracing);

    if (graphManager->allNodesObserved()) {
        RCLCPP_INFO(node->get_logger(), GREEN "[ObserveGraphNodes] SUCCESS — all nodes observed." RESET);
        return BT::NodeStatus::SUCCESS;
    }

    if (visibleNodes.empty()) {
        RCLCPP_INFO(node->get_logger(),
                    ORANGE "[ObserveGraphNodes] No visible unobserved nodes → SUCCESS." RESET);
        return BT::NodeStatus::SUCCESS;
    }

    if (!spinCtrl->shouldTriggerSpin(robotPose, spinDistanceThreshold)) {
        RCLCPP_INFO(node->get_logger(),
                    ORANGE "[ObserveGraphNodes] Robot did not move enough for new spin → SUCCESS." RESET);
        return BT::NodeStatus::SUCCESS;
    }

    bool success = set_remote_parameter(node, valueMapNode, decayFactorParam, 1.0);
    if (!success) 
    {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[ObserveGraphNodes] Warning: could not set decay_factor parameter." RESET);
    }

    double minYaw = 0.0, maxYaw = 0.0;
    if (!visibility->computeYawSpan(visibleNodes, robotPose, minYaw, maxYaw)) {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[ObserveGraphNodes] Could not compute yaw span → RUNNING." RESET);
        return BT::NodeStatus::RUNNING;
    }

    spinCtrl->configure(minYaw, maxYaw);
    spinCtrl->startSpinToMin(mapFrame);

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[ObserveGraphNodes] Initiated spin sequence [%.1f°, %.1f°]" RESET,
                minYaw * 180.0 / M_PI, maxYaw * 180.0 / M_PI);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ObserveGraphNodes::onRunning() {

    // === Timeout check ===
    BT::NodeStatus timeoutStatus;
    if (checkTimeout(timeoutStatus))
        return timeoutStatus;

    auto bestNode = graphManager->getBestScoreNode();
    if (bestNode)
        setOutput("likiest_explorer_node", bestNode);

    auto state = spinCtrl->updateSpinState(mapFrame);
    if (state == BT::NodeStatus::SUCCESS) 
    {
        bool success = set_remote_parameter(node, valueMapNode, decayFactorParam, valueMapDecayFactorDefault);
        if (!success) 
        {
            RCLCPP_WARN(node->get_logger(),
                        YELLOW "[ObserveGraphNodes] Warning: could not reset decay_factor parameter." RESET);
        }
        if (graphManager->allNodesObserved()) 
        {
            RCLCPP_INFO(node->get_logger(), GREEN "[ObserveGraphNodes] SUCCESS — all nodes now observed." RESET);
            return BT::NodeStatus::SUCCESS;
        } 
        else
        {
            RCLCPP_ERROR(node->get_logger(),
                         RED "[ObserveGraphNodes] FAILURE — unobserved nodes remain after full spin." RESET);
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void ObserveGraphNodes::onHalted() {
    RCLCPP_WARN(node->get_logger(),
                YELLOW "[ObserveGraphNodes] Observation halted by Behavior Tree." RESET);
    bool success = set_remote_parameter(node, valueMapNode, decayFactorParam, valueMapDecayFactorDefault);

    // // Gracefully reset all GraphNodeManager subscriptions
    // if (graphManager) {
    //     graphManager->shutdown();
    //     graphManager.reset();
    //     RCLCPP_INFO(node->get_logger(),
    //                 GREEN "[ObserveGraphNodes] GraphNodeManager shutdown completed." RESET);
    // }

    if (spinCtrl)
        spinCtrl->cancel();
}

bool ObserveGraphNodes::checkTimeout(BT::NodeStatus& outStatus)
{
    if (!timerActive)
        return false;

    const double elapsed = (steadyClock.now() - startTime).seconds();

    if (elapsed >= timeoutSec)
    {
        RCLCPP_ERROR(node->get_logger(),
                     RED "[ObserveGraphNodes] TIMEOUT: exceeded %.1f s (elapsed %.1f s) → FAILURE" RESET,
                     timeoutSec, elapsed);


        timerActive = false;
        outStatus = BT::NodeStatus::FAILURE;
        return true;
    }

    return false;
}

bool ObserveGraphNodes::setRemoteParameter(const std::string& targetNode,
                                           const std::string& paramName,
                                           double value,
                                           double timeoutSec)
{
    auto helperNode = std::make_shared<rclcpp::Node>("observe_graph_nodes_param_client");
    auto client = helperNode->create_client<rcl_interfaces::srv::SetParameters>(
        targetNode + "/set_parameters");

    if (!client->wait_for_service(std::chrono::duration<double>(timeoutSec))) {
        RCLCPP_ERROR(node->get_logger(),
                     RED "[ObserveGraphNodes] Parameter service not available: %s" RESET,
                     (targetNode + "/set_parameters").c_str());
        return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(rclcpp::Parameter(paramName, value).to_parameter_msg());

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[ObserveGraphNodes] Setting %s/%s = %.4f" RESET,
                targetNode.c_str(), paramName.c_str(), value);

    auto future = client->async_send_request(request);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(helperNode);

    bool success = false;
    if (exec.spin_until_future_complete(future, std::chrono::duration<double>(timeoutSec)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[ObserveGraphNodes] Parameter %s/%s updated to %.4f" RESET,
                    targetNode.c_str(), paramName.c_str(), value);
        success = true;
    }
    else {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[ObserveGraphNodes] Timeout or failure setting %s/%s" RESET,
                    targetNode.c_str(), paramName.c_str());
    }

    exec.remove_node(helperNode);
    return success;
}

