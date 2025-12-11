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
    navClient = rclcpp_action::create_client<NavigateToPose>(node, "/navigate_to_pose");

    if (!node->has_parameter("go_to_graph_node.timeout"))
        node->declare_parameter<double>("go_to_graph_node.timeout", timeoutSec);
    if (!node->has_parameter("go_to_graph_node.approach_radius"))
        node->declare_parameter<double>("go_to_graph_node.approach_radius", approachRadius);

    timeoutSec = node->get_parameter("go_to_graph_node.timeout").as_double();
    approachRadius = node->get_parameter("go_to_graph_node.approach_radius").as_double();
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
    if (!input)
    {
        RCLCPP_ERROR(node->get_logger(), "[%s] No graph_node input.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    target = input.value();
    approachRadius = getInput<double>("approach_radius").value_or(approachRadius);

    // Wait for Nav2 action server
    if (!navClient->wait_for_action_server(3s))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "[%s] Nav2 action server 'navigate_to_pose' not available.",
                     name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // --- Prepare goal ---
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = mapFrame;

    // Humble: use to_msg() free function
    goal.pose.header.stamp = node->now();

    goal.pose.pose.position = target->position;
    goal.pose.pose.orientation.w = 1.0;

    // --- Send goal asynchronously ---
    goalFuture = navClient->async_send_goal(goal);
    goalSent = true;
    goalDone = false;
    goalSucceeded = false;
    startTime = node->now();

    RCLCPP_INFO(node->get_logger(),
                "[%s] Sent Nav2 goal → (%.2f, %.2f)",
                name().c_str(),
                target->position.x, target->position.y);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToGraphNode::onRunning()
{
    // ============================================================
    // Check distance to target first (Approach Radius)
    // ============================================================
    geometry_msgs::msg::Pose robotPose;
    if (robot->getPose(robotPose, mapFrame, robotFrame))
    {
        double dx = target->position.x - robotPose.position.x;
        double dy = target->position.y - robotPose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist <= approachRadius)
        {
            RCLCPP_INFO(node->get_logger(),
                        "[%s] Within approach radius (%.2f ≤ %.2f). Stopping and succeeding.",
                        name().c_str(), dist, approachRadius);

            // Cancel current Nav2 goal if any
            if (goalHandle)
            {
                try {
                    navClient->async_cancel_goal(goalHandle);
                    RCLCPP_INFO(node->get_logger(),
                                "[%s] Active Nav2 goal canceled due to proximity.", name().c_str());
                } catch (const std::exception &e) {
                    RCLCPP_WARN(node->get_logger(),
                                "[%s] Exception during cancel: %s", name().c_str(), e.what());
                }
                goalHandle.reset();
            }

            // Optionally stop robot directly (safety)
            robot->cancelNavigationGoals();

            goalSent = false;
            goalDone = false;
            goalSucceeded = true;
            return BT::NodeStatus::SUCCESS;
        }
    }

    auto input = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node");
    if (input && target)
    {
        auto newTarget = input.value();
        if (newTarget)
        {
            // Compute positional drift
            double dx = newTarget->position.x - target->position.x;
            double dy = newTarget->position.y - target->position.y;
            double dist = std::hypot(dx, dy);

            bool idChanged = (newTarget->id != target->id);
            bool positionShifted = (dist > 0.5);  // configurable threshold

            if (idChanged || positionShifted)
            {
                RCLCPP_WARN(node->get_logger(),
                            "[%s] Graph node target changed → "
                            "(old ID %d @ %.2f, %.2f → new ID %d @ %.2f, %.2f). Re-sending goal.",
                            name().c_str(),
                            target->id, target->position.x, target->position.y,
                            newTarget->id, newTarget->position.x, newTarget->position.y);

                // Cancel any current Nav2 goal before re-sending
                if (goalHandle)
                {
                    try {
                        navClient->async_cancel_goal(goalHandle);
                        RCLCPP_INFO(node->get_logger(),
                                    "[%s] Previous Nav2 goal canceled.", name().c_str());
                    }
                    catch (const std::exception &e) {
                        RCLCPP_WARN(node->get_logger(),
                                    "[%s] Failed to cancel previous goal cleanly: %s",
                                    name().c_str(), e.what());
                    }
                }

                // Update target and reset internal state
                target = newTarget;
                goalHandle.reset();
                goalSent = false;
                goalDone = false;
                goalSucceeded = false;
                startTime = node->now();

                // Send new Nav2 goal
                NavigateToPose::Goal goal;
                goal.pose.header.frame_id = mapFrame;
                goal.pose.header.stamp = node->now();
                goal.pose.pose.position = target->position;
                goal.pose.pose.orientation.w = 1.0;

                goalFuture = navClient->async_send_goal(goal);
                goalSent = true;

                RCLCPP_INFO(node->get_logger(),
                            "[%s] Sent updated Nav2 goal → (%.2f, %.2f).",
                            name().c_str(),
                            target->position.x, target->position.y);
                return BT::NodeStatus::RUNNING;
            }
        }
    }

    // Wait for goal acceptance
    if (goalSent && !goalHandle)
    {
        if (goalFuture.wait_for(0s) == std::future_status::ready)
        {
            goalHandle = goalFuture.get();
            if (!goalHandle)
            {
                RCLCPP_ERROR(node->get_logger(),
                             "[%s] Goal rejected or invalid handle returned by Nav2.",
                             name().c_str());
                goalSent = false;
                return BT::NodeStatus::FAILURE;
            }

            resultFuture = navClient->async_get_result(goalHandle);
            RCLCPP_INFO(node->get_logger(), "[%s] Goal accepted by Nav2.", name().c_str());
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    // Wait for Nav2 result
    if (goalHandle && !goalDone)
    {
        if (resultFuture.wait_for(0s) == std::future_status::ready)
        {
            auto wrappedResult = resultFuture.get();
            goalDone = true;

            RCLCPP_INFO(node->get_logger(),
                        "[%s] Nav2 returned result code: %d",
                        name().c_str(), static_cast<int>(wrappedResult.code));

            goalSucceeded = (wrappedResult.code == rclcpp_action::ResultCode::SUCCEEDED);
        }
        else
        {
            double elapsed = (node->now() - startTime).seconds();
            if (elapsed > timeoutSec)
            {
                RCLCPP_WARN(node->get_logger(),
                            "[%s] Timeout after %.1f s (limit %.1f). Canceling goal.",
                            name().c_str(), elapsed, timeoutSec);
                if (goalHandle)
                    navClient->async_cancel_goal(goalHandle);
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
    }

    if (goalDone)
    {
        if (goalSucceeded)
        {
            RCLCPP_INFO(node->get_logger(),
                        "[%s] Navigation succeeded to (%.2f, %.2f).",
                        name().c_str(),
                        target->position.x, target->position.y);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(),
                         "[%s] Navigation failed or aborted by Nav2.",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void GoToGraphNode::onHalted()
{
    if (goalHandle)
    {
        try
        {
            navClient->async_cancel_goal(goalHandle);
            RCLCPP_INFO(node->get_logger(),
                        "[%s] Navigation goal canceled.", name().c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(node->get_logger(),
                        "[%s] Exception during cancel: %s",
                        name().c_str(), e.what());
        }
    }
    goalHandle.reset();
    goalSent = false;
    goalDone = false;
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
