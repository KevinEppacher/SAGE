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
        });

    subMap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latestMap = std::move(msg);
        });
}

bool GraphNodeManager::noRecentGraph() {
    if (!receivedGraph) {
        missedTicks++;
        if (missedTicks > maxMissedTicks) {
            return true;
        }
        return true; // waiting but not yet failing
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
    return std::make_shared<graph_node_msgs::msg::GraphNode>(*bestNode);
}

bool GraphNodeManager::allNodesObserved() const {
    if (!latestGraph) return false;
    for (const auto& n : latestGraph->nodes) {
        if (!n.is_observed) return false;
    }
    return true;
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

SpinController::SpinController(std::unique_ptr<Robot>& robotRef)
    : robot(robotRef)
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
}

BT::NodeStatus SpinController::updateSpinState(const std::string& mapFrame) {
    switch (state) {
        case State::SPINNING_MIN:
            if (robot->isSpinDone()) {
                robot->spinRelativeTo(mapFrame, homeYaw, 4.0);
                state = State::SPINNING_HOME;
            }
            break;
        case State::SPINNING_HOME:
            if (robot->isSpinDone()) {
                robot->spinRelativeTo(mapFrame, maxYaw, 4.0);
                state = State::SPINNING_MAX;
            }
            break;
        case State::SPINNING_MAX:
            if (robot->isSpinDone()) {
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

    for (const auto& n : manager.latestGraph->nodes) {
        if (!n.is_observed && rayTracer.isVisible(n, robotPose, manager.latestMap, sightHorizon, performRaytrace))
            result.push_back(&n);
    }
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
    spinCtrl = std::make_unique<SpinController>(robot);
    visibility = std::make_unique<VisibilityAnalyzer>(*rayTracer);

    // Parameter loading
    node->declare_parameter("observe.perform_raytracing", false);
    node->declare_parameter("observe.sight_horizon", 10.0);
    node->declare_parameter("observe.spin_distance_threshold", 1.0);
    node->declare_parameter("observe.map_frame", "map");
    node->declare_parameter("observe.robot_frame", "base_link");

    performRayTracing = node->get_parameter("observe.perform_raytracing").as_bool();
    sightHorizon = node->get_parameter("observe.sight_horizon").as_double();
    spinDistanceThreshold = node->get_parameter("observe.spin_distance_threshold").as_double();
    mapFrame = node->get_parameter("observe.map_frame").as_string();
    robotFrame = node->get_parameter("observe.robot_frame").as_string();
}

BT::PortsList ObserveGraphNodes::providedPorts() {
    return {
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("likiest_explorer_node"),
        BT::OutputPort<bool>("any_exploration_nodes")
    };
}

BT::NodeStatus ObserveGraphNodes::onStart() {
    if (graphManager->noRecentGraph())
        return BT::NodeStatus::RUNNING;

    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose, mapFrame, robotFrame))
        return BT::NodeStatus::RUNNING;

    auto bestNode = graphManager->getBestScoreNode();
    if (bestNode)
        setOutput("likiest_explorer_node", bestNode);

    auto visibleNodes = visibility->findUnobservedVisible(*graphManager, robotPose, sightHorizon, performRayTracing);
    if (graphManager->allNodesObserved())
        return BT::NodeStatus::SUCCESS;

    if (visibleNodes.empty())
        return BT::NodeStatus::RUNNING;

    if (!spinCtrl->shouldTriggerSpin(robotPose, spinDistanceThreshold))
        return BT::NodeStatus::SUCCESS;

    double minYaw = 0.0, maxYaw = 0.0;
    if (!visibility->computeYawSpan(visibleNodes, robotPose, minYaw, maxYaw))
        return BT::NodeStatus::RUNNING;

    spinCtrl->configure(minYaw, maxYaw);
    spinCtrl->startSpinToMin(mapFrame);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ObserveGraphNodes::onRunning() {
    auto bestNode = graphManager->getBestScoreNode();
    if (bestNode)
        setOutput("likiest_explorer_node", bestNode);

    auto state = spinCtrl->updateSpinState(mapFrame);
    if (state == BT::NodeStatus::SUCCESS) {
        if (graphManager->allNodesObserved())
            return BT::NodeStatus::SUCCESS;
        else
            return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
}

void ObserveGraphNodes::onHalted() {
    spinCtrl->cancel();
    RCLCPP_INFO(node->get_logger(), "ObserveGraphNodes halted.");
}
