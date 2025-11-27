#include "sage_behaviour_tree/utils.hpp"
#include "sage_behaviour_tree/colors.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace sage_bt_utils
{

bool set_remote_parameter(const rclcpp::Node::SharedPtr& node,
                          const std::string& target_node,
                          const std::string& param_name,
                          double value,
                          double timeout_sec)
{
    auto helper_node = std::make_shared<rclcpp::Node>("param_client_helper");
    auto client = helper_node->create_client<rcl_interfaces::srv::SetParameters>(
        target_node + "/set_parameters");

    if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
        RCLCPP_ERROR(node->get_logger(),
                     RED "[set_remote_parameter] Parameter service not available for %s" RESET,
                     target_node.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[set_remote_parameter] Setting %s/%s = %.4f" RESET,
                target_node.c_str(), param_name.c_str(), value);

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(rclcpp::Parameter(param_name, value).to_parameter_msg());

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(helper_node);
    auto future = client->async_send_request(request);

    bool success = false;
    if (exec.spin_until_future_complete(future, std::chrono::duration<double>(timeout_sec))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[set_remote_parameter] Successfully updated %s/%s to %.4f" RESET,
                    target_node.c_str(), param_name.c_str(), value);
        success = true;
    }
    else {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[set_remote_parameter] Timeout or failure setting %s/%s" RESET,
                    target_node.c_str(), param_name.c_str());
    }

    exec.remove_node(helper_node);
    return success;
}

}  // namespace sage_bt_utils

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

    bool ok = sage_bt_utils::set_remote_parameter(node_ptr_,
                                                  target_node,
                                                  param_name,
                                                  value);

    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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