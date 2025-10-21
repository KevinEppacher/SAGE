#include "sage_behaviour_tree/navigation.hpp"
#include <chrono>
#include <cmath>

// -------------------- Spin -------------------- //

Spin::Spin(const std::string& name,
           const BT::NodeConfiguration& config,
           rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node))
{
}

BT::PortsList Spin::providedPorts()
{
    return {
        BT::InputPort<double>("turnLeftAngle", 0.0, "CCW rotation angle in radians"),
        BT::InputPort<double>("turnRightAngle", 0.0, "CW rotation angle in radians (negative)"),
        BT::InputPort<double>("spinDuration", 15.0, "Spin duration seconds")
    };
}

double Spin::shortestReturn(double angle)
{
    if (std::fabs(angle) <= M_PI)
        return -angle;
    return (angle > 0.0) ? 2 * M_PI - angle : -2 * M_PI - angle;
}

BT::NodeStatus Spin::onStart()
{
    getInput("turnLeftAngle", turnLeftAngle);
    getInput("turnRightAngle", turnRightAngle);
    getInput("spinDuration", spinDuration);

    // Abort all navigation goals before spinning
    robot->cancelNavigationGoals();

    done = false;
    phase = 0;
    cumulativeRotation = 0.0;

    if (turnLeftAngle == 0.0 && turnRightAngle == 0.0)
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s] Both angles are 0 → nothing to spin.", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // Full-turn optimization
    bool fullLeft = std::fabs(turnLeftAngle) > M_PI;
    bool fullRight = std::fabs(turnRightAngle) > M_PI;
    double fullTurn = 2 * M_PI;

    if (fullLeft && !fullRight)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%s] Performing full CCW turn (%.2f rad)", name().c_str(), turnLeftAngle);
        robot->spin(fullTurn, spinDuration);
        phase = 99;
        return BT::NodeStatus::RUNNING;
    }

    if (fullRight && !fullLeft)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%s] Performing full CW turn (%.2f rad)", name().c_str(), turnRightAngle);
        robot->spin(-fullTurn, spinDuration);
        phase = 99;
        return BT::NodeStatus::RUNNING;
    }

    if (fullLeft && fullRight)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%s] Both angles exceed 180° → defaulting to left turn.", name().c_str());
        robot->spin(fullTurn, spinDuration);
        phase = 99;
        return BT::NodeStatus::RUNNING;
    }

    // Multi-phase sequence
    if (turnLeftAngle != 0.0)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%s] Phase 1: spinning CCW to %.2f rad", name().c_str(), turnLeftAngle);
        robot->spin(turnLeftAngle, spinDuration);
        phase = 1;
    }
    else if (turnRightAngle != 0.0)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%s] Phase 3: spinning CW to %.2f rad", name().c_str(), turnRightAngle);
        robot->spin(turnRightAngle, spinDuration);
        phase = 3;
    }
    else
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Spin::onRunning()
{
    if (!robot->isSpinDone())
        return BT::NodeStatus::RUNNING;

    navResult = robot->getSpinResult();

    if (phase == 99)
    {
        if (navResult == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node->get_logger(), "[%s] Full 360° spin completed.", name().c_str());
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_WARN(node->get_logger(), "[%s] Full spin failed or canceled.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (navResult != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_WARN(node->get_logger(), "[%s] Spin phase failed or canceled.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    switch (phase)
    {
        case 1:
        {
            double returnYaw = shortestReturn(turnLeftAngle);
            cumulativeRotation += std::fabs(turnLeftAngle);
            RCLCPP_INFO(node->get_logger(),
                        "[%s] Phase 2: returning to 0 rad (shortest path %.2f rad)",
                        name().c_str(), returnYaw);
            robot->spin(returnYaw, spinDuration);
            phase = 2;
            return BT::NodeStatus::RUNNING;
        }

        case 2:
        {
            cumulativeRotation += std::fabs(shortestReturn(turnLeftAngle));
            if (cumulativeRotation >= 2 * M_PI - 0.01)
            {
                RCLCPP_INFO(node->get_logger(),
                            "[%s] Already rotated full 360° (%.2f rad) → skipping CW.",
                            name().c_str(), cumulativeRotation);
                return BT::NodeStatus::SUCCESS;
            }

            if (turnRightAngle != 0.0)
            {
                RCLCPP_INFO(node->get_logger(),
                            "[%s] Phase 3: spinning CW to %.2f rad",
                            name().c_str(), turnRightAngle);
                robot->spin(turnRightAngle, spinDuration);
                phase = 3;
                return BT::NodeStatus::RUNNING;
            }
            return BT::NodeStatus::SUCCESS;
        }

        case 3:
        {
            double returnYaw = shortestReturn(turnRightAngle);
            RCLCPP_INFO(node->get_logger(),
                        "[%s] Phase 4: returning to 0 rad (%.2f rad)",
                        name().c_str(), returnYaw);
            robot->spin(returnYaw, spinDuration);
            phase = 4;
            return BT::NodeStatus::RUNNING;
        }

        case 4:
            RCLCPP_INFO(node->get_logger(),
                        "[%s] Spin sequence complete (CCW → 0 → CW → 0).",
                        name().c_str());
            return BT::NodeStatus::SUCCESS;

        default:
            return BT::NodeStatus::FAILURE;
    }
}

void Spin::onHalted()
{
    robot->cancelSpin();
    RCLCPP_INFO(node->get_logger(), "[%s] Spin halted.", name().c_str());
}


// -------------------- GoToGraphNode -------------------- //

GoToGraphNode::GoToGraphNode(const std::string& name,
                             const BT::NodeConfiguration& config,
                             rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node))
{
}

BT::PortsList GoToGraphNode::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "graph_nodes", "Target GraphNode to approach"),
        BT::InputPort<double>("approach_radius", 2.0, "Approach distance in meters"),
        BT::InputPort<std::string>("goal_topic", "/goal_pose", "Goal topic name"),
        BT::InputPort<std::string>("robot_frame", "base_link", "Robot TF frame"),
        BT::InputPort<std::string>("map_frame", "map", "Map TF frame"),
        BT::InputPort<double>("timeout", 60.0, "Timeout in seconds")   // <-- new port
    };
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
        RCLCPP_WARN(node->get_logger(), "[%s] No graph_nodes input.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    target = nodeRes.value();

    if (isWithinGoal(*target))
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Already within %.2f m.", name().c_str(), approachRadius);
        return BT::NodeStatus::SUCCESS;
    }

    robot->cancelNavigationGoals();
    robot->publishGoalToTarget(*target, goalTopic, mapFrame);
    startTime = node->now();
    lastPublishTime = startTime;

    RCLCPP_INFO(node->get_logger(), "[%s] Published goal (%.2f, %.2f), timeout %.1f s.",
                name().c_str(), target->position.x, target->position.y, timeoutSec);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToGraphNode::onRunning()
{
    if (!target)
        return BT::NodeStatus::FAILURE;

    // --- Timeout check ---
    const double elapsed = (node->now() - startTime).seconds();
    RCLCPP_INFO(node->get_logger(), "[%s] Elapsed time: %.1f s until Timeout %.1f s", name().c_str(), elapsed, timeoutSec);
    if (elapsed > timeoutSec && timeoutSec > 0.0)
    {
        RCLCPP_WARN(node->get_logger(), "[%s] Timeout after %.1f s (limit %.1f s).",
                    name().c_str(), elapsed, timeoutSec);
        robot->cancelNavigationGoals();
        return BT::NodeStatus::FAILURE;
    }

    // --- Goal reached check ---
    if (isWithinGoal(*target))
    {
        RCLCPP_INFO(node->get_logger(), "[%s] Target reached within %.2f m.", name().c_str(), approachRadius);
        robot->cancelNavigationGoals();
        return BT::NodeStatus::SUCCESS;
    }

    // --- Periodic re-publish (every 1 s) ---
    const double republishInterval = 1.0;
    const double sinceLastPub = (node->now() - lastPublishTime).seconds();

    if (sinceLastPub >= republishInterval)
    {
        robot->publishGoalToTarget(*target, goalTopic, mapFrame);
        lastPublishTime = node->now();
    }

    return BT::NodeStatus::RUNNING;
}

void GoToGraphNode::onHalted()
{
    robot->cancelNavigationGoals();
    RCLCPP_INFO(node->get_logger(), "[%s] GoToGraphNode halted.", name().c_str());
}

bool GoToGraphNode::isWithinGoal(const graph_node_msgs::msg::GraphNode& goal)
{
    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose, mapFrame, robotFrame))
        return false;

    double dx = goal.position.x - robotPose.position.x;
    double dy = goal.position.y - robotPose.position.y;
    double dist = std::hypot(dx, dy);
    return dist <= approachRadius;
}


// -------------------- RealignToObject -------------------- //

RealignToObject::RealignToObject(const std::string& name,
                                 const BT::NodeConfiguration& config,
                                 rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node))
{
}

BT::PortsList RealignToObject::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "detected_graph_node", "Best detected graph node"),
        BT::InputPort<double>("spinDuration", 5.0, "Duration allowed for spin")
    };
}

BT::NodeStatus RealignToObject::onStart()
{
    auto res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("detected_graph_node");
    getInput("spinDuration", spinDuration);

    if (!res)
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s] No detected_graph_node input provided.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    const auto& target = res.value();
    if (!target)
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s] Received null target node.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose))
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s] Cannot get robot pose.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    targetYaw = computeYawToTarget(robotPose, *target);
    RCLCPP_INFO(node->get_logger(),
                "[%s] Realigning by %.3f rad to node ID %d (score %.2f).",
                name().c_str(), targetYaw, target->id, target->score);

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
        return BT::NodeStatus::RUNNING;

    if (robot->getSpinResult() == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%s] Realign spin completed successfully.", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node->get_logger(),
                "[%s] Realign spin failed or canceled.", name().c_str());
    return BT::NodeStatus::FAILURE;
}

void RealignToObject::onHalted()
{
    if (started)
        robot->cancelSpin();
    started = false;
    RCLCPP_INFO(node->get_logger(), "[%s] Halted.", name().c_str());
}

double RealignToObject::computeYawToTarget(const geometry_msgs::msg::Pose& robotPose,
                                           const graph_node_msgs::msg::GraphNode& target)
{
    double robotYaw = tf2::getYaw(robotPose.orientation);
    double dx = target.position.x - robotPose.position.x;
    double dy = target.position.y - robotPose.position.y;
    double targetYaw = std::atan2(dy, dx);
    double yawDiff = targetYaw - robotYaw;

    // Normalize to [-π, π]
    while (yawDiff > M_PI) yawDiff -= 2 * M_PI;
    while (yawDiff < -M_PI) yawDiff += 2 * M_PI;

    return yawDiff;
}