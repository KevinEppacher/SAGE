#include "sage_behaviour_tree/utils.hpp"
#include "sage_behaviour_tree/colors.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

inline bool cellIsOccupied(const nav_msgs::msg::OccupancyGrid& map, int mx, int my)
{
    if (mx < 0 || my < 0) return false;
    if (mx >= static_cast<int>(map.info.width)) return false;
    if (my >= static_cast<int>(map.info.height)) return false;

    int idx = my * map.info.width + mx;
    int8_t value = map.data[idx];

    return value >= 50;   // standard ROS threshold
}


// -------------------- SetParameterNode -------------------- //

SetParameterNode::SetParameterNode(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name, config),
      node_ptr_(std::move(node_ptr))
{}

BT::PortsList SetParameterNode::providedPorts()
{
    return {
        BT::InputPort<std::string>("target_node", "Node name to set parameter on"),
        BT::InputPort<std::string>("parameter_name", "Name of the parameter to set"),
        BT::InputPort<double>("value", "Value to set for the parameter")
    };
}

BT::NodeStatus SetParameterNode::tick()
{
    std::string target_node, param_name;
    double value;

    if (!getInput("target_node", target_node) ||
        !getInput("parameter_name", param_name) ||
        !getInput("value", value))
    {
        RCLCPP_ERROR(node_ptr_->get_logger(),
                     RED "[%s] Missing required input port(s)" RESET,
                     name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    auto helper_node = std::make_shared<rclcpp::Node>("param_client_helper");
    auto client = helper_node->create_client<rcl_interfaces::srv::SetParameters>(
        target_node + "/set_parameters");

    if (!client->wait_for_service(2s)) {
        RCLCPP_ERROR(node_ptr_->get_logger(),
                     RED "[%s] Parameter service not available for %s" RESET,
                     name().c_str(), target_node.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_ptr_->get_logger(),
                ORANGE "[%s] Setting %s/%s = %.3f" RESET,
                name().c_str(), target_node.c_str(), param_name.c_str(), value);

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(rclcpp::Parameter(param_name, value).to_parameter_msg());

    auto future = client->async_send_request(request);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(helper_node);

    if (exec.spin_until_future_complete(future, 2s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    GREEN "[%s] Successfully updated parameter %s to %.3f" RESET,
                    name().c_str(), param_name.c_str(), value);
        exec.remove_node(helper_node);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    YELLOW "[%s] Timeout or failure setting parameter %s on %s" RESET,
                    name().c_str(), param_name.c_str(), target_node.c_str());
        exec.remove_node(helper_node);
        return BT::NodeStatus::FAILURE;
    }
}


// -------------------- SeekoutGraphNodes -------------------- //

SeekoutGraphNodes::SeekoutGraphNodes(const std::string &name,
                                     const BT::NodeConfiguration &config,
                                     rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      nodePtr_(std::move(nodePtr)),
      robot_(std::make_unique<Robot>(nodePtr_))
{}

BT::PortsList SeekoutGraphNodes::providedPorts()
{
    return {
        BT::InputPort<std::string>("graph_node_topic", "/fused/exploration_graph_nodes/graph_nodes"),
        BT::InputPort<std::string>("map_frame", "map"),
        BT::InputPort<std::string>("robot_frame", "base_link"),
        BT::InputPort<double>("sight_horizon", std::to_string(10.0), "Horizon distance (m)"),
        BT::InputPort<double>("min_yaw_default", std::to_string(-M_PI/2), "Default min yaw (rad)"),
        BT::InputPort<double>("max_yaw_default", std::to_string(M_PI/2), "Default max yaw (rad)"),
        BT::OutputPort<double>("min_yaw"),
        BT::OutputPort<double>("max_yaw")
    };
}

void SeekoutGraphNodes::initSubscription(const std::string &topic)
{
    if (sub_)
        return;

    sub_ = nodePtr_->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        topic, 10,
        [this](graph_node_msgs::msg::GraphNodeArray::SharedPtr msg)
        {
            latestMsg_ = std::move(msg);
            receivedMsg_ = true;
        });

    RCLCPP_INFO(nodePtr_->get_logger(),
                ORANGE "[%s] Subscribed to %s" RESET,
                name().c_str(), topic.c_str());
}

BT::NodeStatus SeekoutGraphNodes::onStart()
{
    getInput("graph_node_topic", topic_);
    getInput("map_frame", mapFrame_);
    getInput("robot_frame", robotFrame_);
    getInput("sight_horizon", horizon_);
    getInput("min_yaw_default", minDef_);
    getInput("max_yaw_default", maxDef_);

    initSubscription(topic_);

    receivedMsg_ = false;
    latestMsg_.reset();
    startTime_ = nodePtr_->now();

    RCLCPP_INFO(nodePtr_->get_logger(),
                ORANGE "[%s] Waiting for GraphNodes and robot pose..." RESET,
                name().c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SeekoutGraphNodes::onRunning()
{
    startTime_ = clock_->now();
    rclcpp::Time now = clock_->now();
    rclcpp::Duration elapsed = now - startTime_;
    double elapsedSec = elapsed.nanoseconds() * 1e-9;

    if (elapsedSec > timeoutSec_)
    {
        RCLCPP_ERROR(nodePtr_->get_logger(),
                     RED "[%s] Timeout (%.1f s) waiting for data (limit %.0f s)" RESET,
                     name().c_str(), elapsedSec, timeoutSec_);
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose robotPose;
    if (!robot_->getPose(robotPose, mapFrame_, robotFrame_))
    {
        RCLCPP_INFO_THROTTLE(nodePtr_->get_logger(), *nodePtr_->get_clock(), 2000,
                             ORANGE "[%s] Robot pose unavailable yet..." RESET,
                             name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    if (!receivedMsg_ || !latestMsg_)
    {
        RCLCPP_INFO_THROTTLE(nodePtr_->get_logger(), *nodePtr_->get_clock(), 2000,
                             ORANGE "[%s] Waiting for GraphNodes..." RESET,
                             name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    double minYaw{minDef_}, maxYaw{maxDef_};
    if (!computeYawRange(robotPose, horizon_, minYaw, maxYaw))
    {
        RCLCPP_WARN(nodePtr_->get_logger(),
                    YELLOW "[%s] No nodes within horizon → using defaults." RESET,
                    name().c_str());
        setOutput("min_yaw", minDef_);
        setOutput("max_yaw", maxDef_);
        return BT::NodeStatus::SUCCESS;
    }

    setOutput("min_yaw", minYaw);
    setOutput("max_yaw", maxYaw);

    RCLCPP_INFO(nodePtr_->get_logger(),
                GREEN "[%s] Computed visible yaw span: [%.1f°, %.1f°]" RESET,
                name().c_str(), minYaw * 180.0 / M_PI, maxYaw * 180.0 / M_PI);

    return BT::NodeStatus::SUCCESS;
}

void SeekoutGraphNodes::onHalted()
{
    RCLCPP_INFO(nodePtr_->get_logger(),
                YELLOW "[%s] Halted." RESET,
                name().c_str());
}

bool SeekoutGraphNodes::computeYawRange(
    const geometry_msgs::msg::Pose &robotPose,
    double sightHorizon,
    double &minYaw,
    double &maxYaw)
{
    if (!latestMsg_)
        return false;

    const double rx = robotPose.position.x;
    const double ry = robotPose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(robotPose.orientation, q);
    double roll, pitch, robotYaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, robotYaw);

    std::vector<double> yaws;
    yaws.reserve(latestMsg_->nodes.size());

    for (const auto &n : latestMsg_->nodes)
    {
        double dx = n.position.x - rx;
        double dy = n.position.y - ry;
        double dist = std::hypot(dx, dy);
        if (dist > sightHorizon)
            continue;

        double globalYaw = std::atan2(dy, dx);
        double relativeYaw = globalYaw - robotYaw;

        if (relativeYaw > M_PI)
            relativeYaw -= 2 * M_PI;
        else if (relativeYaw < -M_PI)
            relativeYaw += 2 * M_PI;

        yaws.push_back(relativeYaw);
    }

    if (yaws.empty())
        return false;

    auto [minIt, maxIt] = std::minmax_element(yaws.begin(), yaws.end());
    minYaw = *minIt;
    maxYaw = *maxIt;

    RCLCPP_DEBUG(nodePtr_->get_logger(),
                 ORANGE "[%s] Local yaw range: min=%.2f°, max=%.2f°" RESET,
                 name().c_str(), minYaw * 180.0 / M_PI, maxYaw * 180.0 / M_PI);

    return true;
}


// -------------------- CallEmptyService -------------------- //

CallEmptyService::CallEmptyService(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr))
{
    clock = node->get_clock();
}

BT::PortsList CallEmptyService::providedPorts()
{
    return {
        BT::InputPort<std::string>(
            "service_name",
            "/clear_exploration_map",
            "Service name for std_srvs/Empty call")
    };
}

BT::NodeStatus CallEmptyService::onStart()
{
    if (!getInput("service_name", serviceName))
    {
        RCLCPP_ERROR(node->get_logger(),
                     RED "[%s] Missing 'service_name' port." RESET,
                     name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    client = node->create_client<std_srvs::srv::Empty>(serviceName);

    if (!client->wait_for_service(5s))
    {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[%s] Service '%s' not available after 5s." RESET,
                    name().c_str(), serviceName.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Calling '%s'..." RESET,
                name().c_str(), serviceName.c_str());

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto futureRequest = client->async_send_request(request);
    future = futureRequest.future.share();

    accumulatedSeconds = 0.0;
    lastTick = clock->now();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallEmptyService::onRunning()
{
    rclcpp::Time now = clock->now();
    accumulatedSeconds += (now - lastTick).seconds();
    lastTick = now;

    if (accumulatedSeconds > 5.0)
    {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[%s] Timeout while calling '%s' (%.1f s)." RESET,
                    name().c_str(), serviceName.c_str(), accumulatedSeconds);
        return BT::NodeStatus::FAILURE;
    }

    if (future.wait_for(0s) == std::future_status::ready)
    {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[%s] Successfully called '%s'." RESET,
                    name().c_str(), serviceName.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                         ORANGE "[%s] Waiting for service '%s'..." RESET,
                         name().c_str(), serviceName.c_str());
    return BT::NodeStatus::RUNNING;
}

void CallEmptyService::onHalted()
{
    RCLCPP_INFO(node->get_logger(),
                YELLOW "[%s] Halted while waiting for '%s'." RESET,
                name().c_str(), serviceName.c_str());
}

// -------------------- ObserveGraphNodes -------------------- //

ObserveGraphNodes::ObserveGraphNodes(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_unique<Robot>(node))
{
    // If not defined, declare and get parameters with defaults
    if(!node->has_parameter("observe.perform_raytracing"))
        node->declare_parameter("observe.perform_raytracing", false);
    if(!node->has_parameter("observe.sight_horizon"))
        node->declare_parameter("observe.sight_horizon", 10.0);
    if(!node->has_parameter("observe.graph_node_topic"))
        node->declare_parameter("observe.graph_node_topic", graphNodeTopic);
    if(!node->has_parameter("observe.map_frame"))
        node->declare_parameter("observe.map_frame", mapFrame);
    if(!node->has_parameter("observe.robot_frame"))
        node->declare_parameter("observe.robot_frame", robotFrame);
    if(!node->has_parameter("observe.spin_distance_threshold"))
        node->declare_parameter("observe.spin_distance_threshold", 1.0);
        
    performRayTracing = node->get_parameter("observe.perform_raytracing").as_bool();
    sightHorizon = node->get_parameter("observe.sight_horizon").as_double();
    graphNodeTopic = node->get_parameter("observe.graph_node_topic").as_string();
    mapFrame = node->get_parameter("observe.map_frame").as_string();
    robotFrame = node->get_parameter("observe.robot_frame").as_string();
    spinDistanceThreshold = node->get_parameter("observe.spin_distance_threshold").as_double();

    // ----------- GraphNodes Subscriber -----------
    subGraph = node->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        graphNodeTopic, 10,
        [this](graph_node_msgs::msg::GraphNodeArray::SharedPtr msg)
        {
            latestGraph = std::move(msg);
            haveGraph = true;
            receivedGraph = true;
            missedGraphTicks = 0;
        });

    // ----------- Map Subscriber -----------
    subMap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            latestMap = std::move(msg);
            haveMap = true;
        });
}

BT::PortsList ObserveGraphNodes::providedPorts()
{
    return {
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("likiest_explorer_node"),
        BT::OutputPort<bool>("any_exploration_nodes")
    };
}

BT::NodeStatus ObserveGraphNodes::onStart()
{
    scan = {};

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Start observing graph nodes..." RESET,
                name().c_str());

    // --- Handle missing messages ---
    if (!receivedGraph)
    {
        missedGraphTicks++;
        RCLCPP_INFO(node->get_logger(),
                    ORANGE "[%s] Waiting for graph nodes on %s... (%d/%d)" RESET,
                    name().c_str(), graphNodeTopic.c_str(), missedGraphTicks, maxMissedTicks);

        if (missedGraphTicks > maxMissedTicks)
        {
            RCLCPP_WARN(node->get_logger(),
                        RED "[%s] No graph nodes received for %d ticks → FAILURE" RESET,
                        name().c_str(), missedGraphTicks);
            setOutput("any_exploration_nodes", false);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    receivedGraph = false;
    missedGraphTicks = 0;

    if (!latestGraph || latestGraph->nodes.empty())
    {
        RCLCPP_WARN(node->get_logger(),
                    RED "[%s] No graph nodes → FAILURE" RESET,
                    name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    const size_t total = latestGraph->nodes.size();
    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Received %zu graph nodes" RESET,
                name().c_str(), total);

    // --- Get robot pose ---
    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose, mapFrame, robotFrame))
    {
        RCLCPP_INFO(node->get_logger(),
                    ORANGE "[%s] Robot pose not available yet..." RESET,
                    name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    // --- Select and output the likiest explorer node ---
    auto bestNode = findBestScoringNode();
    if (bestNode)
    {
        setOutput("likiest_explorer_node", bestNode);
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[%s] Best-scoring node ID: %d (score = %.3f)" RESET,
                    name().c_str(), bestNode->id, bestNode->score);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[%s] No valid node to output as likiest_explorer_node" RESET,
                    name().c_str());
    }

    // --- Collect unobserved visible nodes ---
    std::vector<const graph_node_msgs::msg::GraphNode*> unobservedVisible;
    int unobservedTotal = 0;
    for (const auto& n : latestGraph->nodes)
    {
        if (!n.is_observed)
        {
            unobservedTotal++;
            if (isVisible(n, robotPose))
                unobservedVisible.push_back(&n);
        }
    }

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Unobserved nodes total: %d, visible: %zu" RESET,
                name().c_str(), unobservedTotal, unobservedVisible.size());

    // --- No unobserved nodes at all → success ---
    if (unobservedTotal == 0)
    {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[%s] All graph nodes already observed → SUCCESS" RESET,
                    name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // --- Some unobserved nodes exist ---
    if (unobservedVisible.empty())
    {
        RCLCPP_INFO(node->get_logger(),
                    YELLOW "[%s] Unobserved nodes exist but none visible → RUNNING" RESET,
                    name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    // --- Spin gating ---
    if (!shouldTriggerSpin(robotPose))
    {
        RCLCPP_INFO(node->get_logger(),
                    YELLOW "[%s] Robot did not move enough → RUNNING" RESET,
                    name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // --- Compute yaw span ---
    double minYaw = 0.0;
    double maxYaw = 0.0;

    if (!computeVisibleYawSpan(minYaw, maxYaw))
    {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[%s] No visible unobserved nodes for yaw span → RUNNING" RESET,
                    name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    scan.minYaw = minYaw;
    scan.maxYaw = maxYaw;

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Yaw span [%.1f°, %.1f°]" RESET,
                name().c_str(), minYaw * 180.0 / M_PI, maxYaw * 180.0 / M_PI);

    // --- Spin to minYaw (absolute) ---
    robot->spinRelativeTo(mapFrame, scan.minYaw, 4.0);
    scan.spinningToMin = true;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ObserveGraphNodes::onRunning()
{
    // Continuously output the current likiest node
    auto bestNode = findBestScoringNode();
    if (bestNode)
        setOutput("likiest_explorer_node", bestNode);

    // Phase 1: Spin to MIN yaw
    if (scan.spinningToMin)
    {
        if (robot->isSpinDone())
        {
            scan.spinningToMin = false;
            scan.finishedMin = true;

            RCLCPP_INFO(node->get_logger(),
                        GREEN "[%s] Finished spin to MIN yaw → returning home" RESET,
                        name().c_str());

            // Compute home yaw as the midpoint between min and max
            scan.homeYaw = (scan.minYaw + scan.maxYaw) / 2.0;

            // Spin back to home
            robot->spinRelativeTo(mapFrame, scan.homeYaw, 4.0);
            scan.spinningHome = true;
        }
        return BT::NodeStatus::RUNNING;
    }

    // Phase 2: Spin HOME
    if (scan.spinningHome)
    {
        if (robot->isSpinDone())
        {
            scan.spinningHome = false;
            scan.finishedHome = true;

            RCLCPP_INFO(node->get_logger(),
                        GREEN "[%s] Returned to home yaw → spinning to MAX yaw" RESET,
                        name().c_str());

            // Now spin left (maxYaw)
            robot->spinRelativeTo(mapFrame, scan.maxYaw, 4.0);
            scan.spinningToMax = true;
        }
        return BT::NodeStatus::RUNNING;
    }

    // Phase 3: Spin to MAX yaw
    if (scan.spinningToMax)
    {
        if (robot->isSpinDone())
        {
            scan.spinningToMax = false;
            scan.finishedMax = true;
            RCLCPP_INFO(node->get_logger(),
                        GREEN "[%s] Finished spin to MAX yaw — checking observations" RESET,
                        name().c_str());

            if (!latestGraph)
                return BT::NodeStatus::RUNNING;

            bool allObserved = true;
            for (const auto& n : latestGraph->nodes)
            {
                if (!n.is_observed)
                {
                    allObserved = false;
                    break;
                }
            }

            if (allObserved)
            {
                RCLCPP_INFO(node->get_logger(),
                            GREEN "[%s] All graph nodes observed → SUCCESS" RESET,
                            name().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_WARN(node->get_logger(),
                            YELLOW "[%s] Still unobserved nodes remaining → RUNNING" RESET,
                            name().c_str());
                return BT::NodeStatus::RUNNING;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    // Default: keep running until spin sequences are complete
    return BT::NodeStatus::RUNNING;
}

void ObserveGraphNodes::onHalted()
{
    robot->cancelSpin();
    RCLCPP_INFO(node->get_logger(),
                YELLOW "[%s] Spin halted by Behavior Tree." RESET,
                name().c_str());
}

bool ObserveGraphNodes::shouldTriggerSpin(const geometry_msgs::msg::Pose& pose)
{
    if (!lastSpinPoseValid)
    {
        lastSpinPose = pose;
        lastSpinPoseValid = true;
        return true;
    }

    double dx = pose.position.x - lastSpinPose.position.x;
    double dy = pose.position.y - lastSpinPose.position.y;

    if (std::hypot(dx, dy) > spinDistanceThreshold)
    {
        lastSpinPose = pose;
        return true;
    }

    return false;
}

bool ObserveGraphNodes::computeVisibleYawSpan(double& outMinYaw, double& outMaxYaw)
{
    if (!latestGraph)
        return false;

    geometry_msgs::msg::Pose pose;
    if (!robot->getPose(pose, mapFrame, robotFrame))
        return false;

    const double rx = pose.position.x;
    const double ry = pose.position.y;

    std::vector<double> yaws;
    yaws.reserve(latestGraph->nodes.size());

    for (const auto& n : latestGraph->nodes)
    {
        // Only UNOBSERVED nodes
        if (n.is_observed)
            continue;

        // Must be visible (raytracing + distance)
        if (!isVisible(n, pose))
            continue;

        double dx = n.position.x - rx;
        double dy = n.position.y - ry;

        // ABSOLUTE yaw in MAP frame
        double globalYaw = std::atan2(dy, dx);

        yaws.push_back(globalYaw);
    }

    if (yaws.empty())
        return false;

    auto [minIt, maxIt] = std::minmax_element(yaws.begin(), yaws.end());
    outMinYaw = *minIt;
    outMaxYaw = *maxIt;

    return true;
}

bool ObserveGraphNodes::isVisible(
    const graph_node_msgs::msg::GraphNode& n,
    const geometry_msgs::msg::Pose& pose)
{
    double dx = n.position.x - pose.position.x;
    double dy = n.position.y - pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist > sightHorizon)
        return false;

    if (!performRayTracing || !latestMap)
        return true;

    // Bresenham raytracing
    const auto& map = *latestMap;

    const double res = map.info.resolution;
    const double originX = map.info.origin.position.x;
    const double originY = map.info.origin.position.y;

    auto worldToMap = [&](double wx, double wy, int& mx, int& my)
    {
        mx = static_cast<int>((wx - originX) / res);
        my = static_cast<int>((wy - originY) / res);
    };

    int rx, ry, tx, ty;
    worldToMap(pose.position.x, pose.position.y, rx, ry);
    worldToMap(n.position.x,     n.position.y,     tx, ty);

    if (rx < 0 || ry < 0 || tx < 0 || ty < 0 ||
        rx >= (int)map.info.width || ry >= (int)map.info.height ||
        tx >= (int)map.info.width || ty >= (int)map.info.height)
    {
        return true;
    }

    int x0 = rx, y0 = ry;
    int x1 = tx, y1 = ty;

    int dx_i = std::abs(x1 - x0);
    int dy_i = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx_i - dy_i;

    while (true)
    {
        if (!(x0 == rx && y0 == ry))
        {
            if (cellIsOccupied(map, x0, y0))
                return false;
        }

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;

        if (e2 > -dy_i)
        {
            err -= dy_i;
            x0 += sx;
        }
        if (e2 < dx_i)
        {
            err += dx_i;
            y0 += sy;
        }
    }

    return true;
}

std::shared_ptr<graph_node_msgs::msg::GraphNode> ObserveGraphNodes::findBestScoringNode() const
{
    if (!latestGraph || latestGraph->nodes.empty())
        return nullptr;

    const graph_node_msgs::msg::GraphNode* bestNode = nullptr;
    double bestScore = -std::numeric_limits<double>::infinity();

    for (const auto& n : latestGraph->nodes)
    {
        if (n.score > bestScore)
        {
            bestScore = n.score;
            bestNode = &n;
        }
    }

    if (!bestNode)
        return nullptr;

    // Copy into a shared_ptr so it can be safely passed through BT port
    return std::make_shared<graph_node_msgs::msg::GraphNode>(*bestNode);
}
