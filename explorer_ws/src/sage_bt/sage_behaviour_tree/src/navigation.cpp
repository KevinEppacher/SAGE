#include "sage_behaviour_tree/navigation.hpp"
#include "sage_behaviour_tree/colors.hpp"
#include <chrono>
#include <cmath>

using std::placeholders::_1;

// -------------------- Spin -------------------- //

Spin::Spin(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node))
{
    if (!node->has_parameter("spin_node.spin_timeout"))
        node->declare_parameter<double>("spin_node.spin_timeout", spinTimeout);
    if (!node->has_parameter("spin_node.spin_distance_threshold"))
        node->declare_parameter<double>("spin_node.spin_distance_threshold", spinDistanceThreshold);

    spinTimeout = node->get_parameter("spin_node.spin_timeout").as_double();
    spinDistanceThreshold = node->get_parameter("spin_node.spin_distance_threshold").as_double();
}


BT::PortsList Spin::providedPorts()
{
    return {
        BT::InputPort<double>("turn_left_angle", 0.0, "CCW angle (radians)"),
        BT::InputPort<double>("turn_right_angle", 0.0, "CW angle (radians)"),
        BT::InputPort<double>("spin_duration", 10.0, "Duration per spin")
    };
}

BT::NodeStatus Spin::onStart()
{
    getInput("turn_left_angle", turnLeftAngle);
    getInput("turn_right_angle", turnRightAngle);
    getInput("spin_duration", spinDuration);

    startTimeSteady = steadyClock.now();
    robot->cancelNavigationGoals();

    geometry_msgs::msg::Pose pose;
    if (!robot->getPose(pose, "map", "base_link"))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "%s[%s] Failed to get robot pose → %sFAILURE%s",
                     RED, name().c_str(), RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Point currentPos = pose.position;
    originYaw = tf2::getYaw(pose.orientation);

    if (isNearLastSpin(currentPos) && skippedSpins <= 5)
    {
        skippedSpins++;
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] Skipping spin: within %.2f m of last spin → %sSUCCESS%s",
                    YELLOW, name().c_str(), spinDistanceThreshold, GREEN, RESET);
        return BT::NodeStatus::SUCCESS;
    }


    // Record new spin position
    lastSpinPosition = currentPos;

    // Precompute yaw targets
    leftTarget  = originYaw + std::fabs(turnLeftAngle);
    rightTarget = originYaw - std::fabs(turnRightAngle);

    // Start first phase (left spin)
    phase = 1;
    skippedSpins = 0;
    robot->spinRelativeTo("map", leftTarget, spinDuration);

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Starting spin (yaw=%.2f rad)...%s",
                BLUE, name().c_str(), originYaw, RESET);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Spin::onRunning()
{
    double elapsed = (steadyClock.now() - startTimeSteady).seconds();
    if (elapsed > spinTimeout)
    {
        robot->cancelSpin();
        RCLCPP_ERROR(node->get_logger(),
                     "%s[%s] Spin timeout (%.1f s) → %sFAILURE%s",
                     RED, name().c_str(), elapsed, RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    if (!robot->isSpinDone())
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "%s[%s] Spinning since %.1f / %.1f s → %sRUNNING%s",
                             ORANGE, name().c_str(), elapsed, spinTimeout, ORANGE, RESET);
        return BT::NodeStatus::RUNNING;
    }

    if (robot->getSpinResult() != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] Spin action failed → %sFAILURE%s",
                    RED, name().c_str(), RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    if (phase == 1)
    {
        phase = 2;
        robot->spinRelativeTo("map", originYaw, spinDuration);
        return BT::NodeStatus::RUNNING;
    }

    if (phase == 2)
    {
        phase = 3;
        robot->spinRelativeTo("map", rightTarget, spinDuration);
        return BT::NodeStatus::RUNNING;
    }

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Spin completed → %sSUCCESS%s",
                GREEN, name().c_str(), GREEN, RESET);
    return BT::NodeStatus::SUCCESS;
}

void Spin::onHalted()
{
    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Spin halted.%s",
                YELLOW, name().c_str(), RESET);
    robot->cancelSpin();
}

bool Spin::isNearLastSpin(const geometry_msgs::msg::Point &pos) const
{
    if (!lastSpinPosition.has_value())
        return false;

    const auto &p = lastSpinPosition.value();
    const double dx = pos.x - p.x;
    const double dy = pos.y - p.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    return dist < spinDistanceThreshold;
}

// -------------------- GoToGraphNode -------------------- //

GoToGraphNode::GoToGraphNode(const std::string &name,
                             const BT::NodeConfiguration &config,
                             rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node))
{
    // Declare parameters if not existing
    if (!node->has_parameter("go_to_graph_node.timeout"))
        node->declare_parameter<double>("go_to_graph_node.timeout", timeoutSec);
    if (!node->has_parameter("go_to_graph_node.goal_topic"))
        node->declare_parameter<std::string>("go_to_graph_node.goal_topic", goalTopic);
    if (!node->has_parameter("go_to_graph_node.robot_frame"))
        node->declare_parameter<std::string>("go_to_graph_node.robot_frame", robotFrame);
    if (!node->has_parameter("go_to_graph_node.map_frame"))
        node->declare_parameter<std::string>("go_to_graph_node.map_frame", mapFrame);

    timeoutSec = node->get_parameter("go_to_graph_node.timeout").as_double();
    goalTopic = node->get_parameter("go_to_graph_node.goal_topic").as_string();
    robotFrame = node->get_parameter("go_to_graph_node.robot_frame").as_string();
    mapFrame   = node->get_parameter("go_to_graph_node.map_frame").as_string();
}

BT::PortsList GoToGraphNode::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node"),
        BT::InputPort<double>("approach_radius")
    };
}

BT::NodeStatus GoToGraphNode::onStart()
{
    auto input = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node");
    approachRadius = getInput<double>("approach_radius").value_or(approachRadius);

    if (!input)
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] No graph_node input → %sFAILURE%s",
                    RED, name().c_str(), RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    target = input.value();
    target->position.z = 0.0;
    lastTargetSnapshot = *target;

    robot->cancelNavigationGoals();
    robot->publishGoalToTarget(*target, goalTopic, mapFrame);

    startTime = node->now();
    lastPublishTime = startTime;

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Initial goal → (%.2f, %.2f)%s",
                BLUE, name().c_str(),
                target->position.x, target->position.y,
                RESET);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToGraphNode::onRunning()
{
    // --- Read latest input ---
    auto input = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node");
    approachRadius = getInput<double>("approach_radius").value_or(approachRadius);
    if (!input)
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] Lost graph_node input → %sFAILURE%s",
                    RED, name().c_str(), RED, RESET);
        return BT::NodeStatus::FAILURE;
    }

    auto newTarget = input.value();
    newTarget->position.z = 0.0;

    // --- Detect target change ---
    if (hasTargetChanged(*newTarget))
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] GraphNode changed → Switching to new target (%.2f, %.2f)%s",
                    ORANGE, name().c_str(),
                    newTarget->position.x, newTarget->position.y,
                    RESET);

        target = newTarget;
        lastTargetSnapshot = *newTarget;

        robot->cancelNavigationGoals();
        robot->publishGoalToTarget(*target, goalTopic, mapFrame);

        startTime = node->now();
        lastPublishTime = startTime;

        return BT::NodeStatus::RUNNING;
    }

    // --- Timeout check ---
    double elapsed = (node->now() - startTime).seconds();
    if (elapsed > timeoutSec)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "%s[%s] Timeout after %.1f s (limit %.1f s) → %sFAILURE%s",
                     RED, name().c_str(), elapsed, timeoutSec, RED, RESET);
        robot->cancelNavigationGoals();
        return BT::NodeStatus::FAILURE;
    }

    // --- Check if goal reached ---
    if (isWithinGoal(*target))
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Goal reached → %sSUCCESS%s",
                    GREEN, name().c_str(), GREEN, RESET);
        robot->cancelNavigationGoals();
        return BT::NodeStatus::SUCCESS;
    }

    // --- Periodic republish ---
    if ((node->now() - lastPublishTime).seconds() > 1.0)
    {
        robot->publishGoalToTarget(*target, goalTopic, mapFrame);
        lastPublishTime = node->now();

        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "%s[%s] Re-publishing goal (%.2f, %.2f) → %sRUNNING%s",
                             ORANGE, name().c_str(),
                             target->position.x, target->position.y,
                             ORANGE, RESET);
    }

    return BT::NodeStatus::RUNNING;
}

void GoToGraphNode::onHalted()
{
    robot->cancelNavigationGoals();
    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Navigation halted.%s",
                YELLOW, name().c_str(), RESET);
}


bool GoToGraphNode::hasTargetChanged(const graph_node_msgs::msg::GraphNode &new_target)
{
    const double dx = new_target.position.x - lastTargetSnapshot.position.x;
    const double dy = new_target.position.y - lastTargetSnapshot.position.y;

    // If target moved more than 5cm → treat as new
    return std::hypot(dx, dy) > 0.05;
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
    sleep(0.5);
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
