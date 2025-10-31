#include "sage_behaviour_tree/navigation.hpp"
#include "sage_behaviour_tree/colors.hpp"
#include <chrono>
#include <cmath>

// -------------------- Spin -------------------- //

Spin::Spin(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node)) 
{
    // Declare ROS parameter for timeout if not already present
    if (!node->has_parameter("spin_node.spin_timeout"))
        node->declare_parameter<double>("spin_node.spin_timeout", spinTimeout);
    // if (!node->has_parameter("spin_node.spin_timeout"))
    //     node->declare_parameter<double>("spin_node.spin_timeout", spinTimeout);
        
    spinTimeout = node->get_parameter("spin_node.spin_timeout").as_double();
}

BT::PortsList Spin::providedPorts()
{
    return {
        BT::InputPort<double>("turn_left_angle", 0.0, "CCW rotation angle in radians"),
        BT::InputPort<double>("turn_right_angle", 0.0, "CW rotation angle in radians"),
        BT::InputPort<double>("spin_duration", 15.0, "Spin duration seconds")};
}

double Spin::shortestReturn(double angle)
{
    if (std::fabs(angle) <= M_PI)
        return -angle;
    return (angle > 0.0) ? 2 * M_PI - angle : -2 * M_PI - angle;
}

BT::NodeStatus Spin::onStart()
{
    getInput("turn_left_angle",  turnLeftAngle);
    getInput("turn_right_angle", turnRightAngle);
    getInput("spin_duration",    spinDuration);

    spinTimeout = node->get_parameter("spin_node.spin_timeout").as_double();

    robot->cancelNavigationGoals();
    phase = 0;
    startTimeSteady = steadyClock.now();

    // --- full spin check ---
    bool fullTurnNeeded =
        std::fabs(turnLeftAngle) > 120.0 * M_PI / 180.0 ||
        std::fabs(turnRightAngle) > 120.0 * M_PI / 180.0;

    if (fullTurnNeeded)
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Large yaw span → full 360° spin%s",
                    ORANGE, name().c_str(), RESET);
        robot->spin(2 * M_PI, spinDuration);
        phase = 99;
        return BT::NodeStatus::RUNNING;
    }

    // --- phase 1: spin left ---
    if (turnLeftAngle != 0.0)
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Phase 1: spinning CCW to %.2f rad%s",
                    ORANGE, name().c_str(), turnLeftAngle, RESET);
        robot->spin(turnLeftAngle, spinDuration);
        phase = 1;
        return BT::NodeStatus::RUNNING;
    }

    // --- if no left rotation, go straight to phase 2 ---
    if (turnRightAngle != 0.0)
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] No left angle, starting CW sweep of %.2f rad%s",
                    ORANGE, name().c_str(), turnRightAngle, RESET);
        robot->spin(turnRightAngle, spinDuration);
        phase = 2;
        return BT::NodeStatus::RUNNING;
    }

    RCLCPP_WARN(node->get_logger(),
                "%s[%s] No rotation specified → %sSUCCESS%s",
                GREEN, name().c_str(), GREEN, RESET);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Spin::onRunning()
{
    double elapsed = (steadyClock.now() - startTimeSteady).seconds();
    if (elapsed > spinTimeout)
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] Spin timeout after %.2f s (limit %.2f s) → %sFAILURE%s",
                    RED, name().c_str(), elapsed, spinTimeout, RED, RESET);
        robot->cancelSpin();
        return BT::NodeStatus::FAILURE;
    }

    if (!robot->isSpinDone())
        return BT::NodeStatus::RUNNING;

    navResult = robot->getSpinResult();
    if (navResult != rclcpp_action::ResultCode::SUCCEEDED)
        return BT::NodeStatus::FAILURE;

    switch (phase)
    {
        case 1:
        {
            // from CCW end → CW side relative to origin
            double cwSpan = -(turnLeftAngle + std::fabs(turnRightAngle));
            RCLCPP_INFO(node->get_logger(),
                        "%s[%s] Phase 2: sweeping CW by %.2f rad%s",
                        ORANGE, name().c_str(), cwSpan, RESET);
            robot->spin(cwSpan, spinDuration);
            phase = 2;
            return BT::NodeStatus::RUNNING;
        }

        case 2:
        {
            // from CW end → shortest return to origin
            double returnYaw = shortestReturn(turnRightAngle);
            RCLCPP_INFO(node->get_logger(),
                        "%s[%s] Phase 3: returning %.2f rad to origin%s",
                        ORANGE, name().c_str(), returnYaw, RESET);
            robot->spin(returnYaw, spinDuration);
            phase = 3;
            return BT::NodeStatus::RUNNING;
        }

        case 3:
        case 99:
        {
            RCLCPP_INFO(node->get_logger(),
                        "%s[%s] Spin sequence complete → %sSUCCESS%s",
                        GREEN, name().c_str(), GREEN, RESET);
            return BT::NodeStatus::SUCCESS;
        }

        default:
            return BT::NodeStatus::SUCCESS;
    }
}

void Spin::onHalted()
{
    robot->cancelSpin();
    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Spin halted.%s",
                YELLOW, name().c_str(), RESET);
}


// -------------------- GoToGraphNode -------------------- //

GoToGraphNode::GoToGraphNode(const std::string &name,
                             const BT::NodeConfiguration &config,
                             rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node)) {}

BT::PortsList GoToGraphNode::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes", "Target GraphNode"),
        BT::InputPort<double>("approach_radius", 2.0, "Approach distance in meters"),
        BT::InputPort<std::string>("goal_topic", "/goal_pose", "Goal topic"),
        BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame"),
        BT::InputPort<std::string>("map_frame", "map", "Map frame"),
        BT::InputPort<double>("timeout", 60.0, "Timeout seconds")};
}

BT::NodeStatus GoToGraphNode::onStart()
{
    getInput("approach_radius", approachRadius);
    getInput("goal_topic", goalTopic);
    getInput("map_frame", mapFrame);
    getInput("robot_frame", robotFrame);
    getInput("timeout", timeoutSec);

    auto nodeRes = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes");
    if (!nodeRes)
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] No graph_nodes input → %sFAILURE%s",
                    RED, name().c_str(), RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    target = nodeRes.value();
    target->position.z = 0.0;  // ground the target
    robot->cancelNavigationGoals();

    robot->publishGoalToTarget(*target, goalTopic, mapFrame);
    startTime = node->now();
    lastPublishTime = startTime;

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Published goal (%.2f, %.2f) timeout %.1f s%s",
                ORANGE, name().c_str(), target->position.x, target->position.y, timeoutSec, RESET);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToGraphNode::onRunning()
{
    if (!target)
        return BT::NodeStatus::FAILURE;

    const double elapsed = (node->now() - startTime).seconds();
    if (elapsed > timeoutSec)
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] Timeout %.1f s → %sFAILURE%s",
                    RED, name().c_str(), elapsed, RED, RESET);
        robot->cancelNavigationGoals();
        return BT::NodeStatus::FAILURE;
    }

    if (isWithinGoal(*target))
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Target reached → %sSUCCESS%s",
                    GREEN, name().c_str(), GREEN, RESET);
        robot->cancelNavigationGoals();
        return BT::NodeStatus::SUCCESS;
    }

    double repubInterval = 1.0;
    if ((node->now() - lastPublishTime).seconds() > repubInterval)
    {
        robot->publishGoalToTarget(*target, goalTopic, mapFrame);
        lastPublishTime = node->now();
    }

    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                         "%s[%s] Navigating... %sRUNNING%s",
                         ORANGE, name().c_str(), ORANGE, RESET);
    return BT::NodeStatus::RUNNING;
}

void GoToGraphNode::onHalted()
{
    robot->cancelNavigationGoals();
    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Navigation halted.%s",
                YELLOW, name().c_str(), RESET);
}

bool GoToGraphNode::isWithinGoal(const graph_node_msgs::msg::GraphNode &goal)
{
    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose, mapFrame, robotFrame))
        return false;

    double dx = goal.position.x - robotPose.position.x;
    double dy = goal.position.y - robotPose.position.y;
    return std::hypot(dx, dy) <= approachRadius;
}


// -------------------- RealignToObject -------------------- //

RealignToObject::RealignToObject(const std::string &name,
                                 const BT::NodeConfiguration &config,
                                 rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node)) {}

BT::PortsList RealignToObject::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("detected_graph_node", "Detected node"),
        BT::InputPort<double>("spinDuration", 5.0, "Spin duration seconds")};
}

BT::NodeStatus RealignToObject::onStart()
{
    auto res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("detected_graph_node");
    getInput("spinDuration", spinDuration);

    if (!res || !res.value())
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] No detected_graph_node provided → %sFAILURE%s",
                    RED, name().c_str(), RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose))
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] Cannot get robot pose → %sFAILURE%s",
                    RED, name().c_str(), RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    const auto &targetNode = *res.value();
    targetYaw = computeYawToTarget(robotPose, targetNode);

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Realigning by %.3f rad to node %d (score %.2f)%s",
                ORANGE, name().c_str(), targetYaw, targetNode.id, targetNode.score, RESET);

    robot->cancelNavigationGoals();
    robot->spin(targetYaw, spinDuration);
    started = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RealignToObject::onRunning()
{
    if (!started)
        return BT::NodeStatus::FAILURE;

    if (!robot->isSpinDone())
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "%s[%s] Aligning... %sRUNNING%s",
                             ORANGE, name().c_str(), ORANGE, RESET);
        return BT::NodeStatus::RUNNING;
    }

    if (robot->getSpinResult() == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Alignment complete → %sSUCCESS%s",
                    GREEN, name().c_str(), GREEN, RESET);
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node->get_logger(),
                "%s[%s] Alignment failed or canceled → %sFAILURE%s",
                RED, name().c_str(), RED, RESET);
    return BT::NodeStatus::FAILURE;
}

void RealignToObject::onHalted()
{
    if (started)
        robot->cancelSpin();
    started = false;
    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Realign halted.%s",
                YELLOW, name().c_str(), RESET);
}

double RealignToObject::computeYawToTarget(const geometry_msgs::msg::Pose &robotPose,
                                           const graph_node_msgs::msg::GraphNode &target)
{
    double robotYaw = tf2::getYaw(robotPose.orientation);
    double dx = target.position.x - robotPose.position.x;
    double dy = target.position.y - robotPose.position.y;
    double targetYaw = std::atan2(dy, dx);
    double yawDiff = targetYaw - robotYaw;
    while (yawDiff > M_PI)
        yawDiff -= 2 * M_PI;
    while (yawDiff < -M_PI)
        yawDiff += 2 * M_PI;
    return yawDiff;
}
