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

    if (isNearLastSpin(currentPos))
    {
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
                             "%s[%s] Spinning... %sRUNNING%s",
                             ORANGE, name().c_str(), ORANGE, RESET);
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
    if (!node->has_parameter("go_to_graph_node.timeout"))
        node->declare_parameter<double>("go_to_graph_node.timeout", timeoutSec);
    if (!node->has_parameter("go_to_graph_node.goal_topic"))
        node->declare_parameter<std::string>("go_to_graph_node.goal_topic", "/goal_pose");
    if (!node->has_parameter("go_to_graph_node.robot_frame"))
        node->declare_parameter<std::string>("go_to_graph_node.robot_frame", "base_link");
    if (!node->has_parameter("go_to_graph_node.map_frame"))
        node->declare_parameter<std::string>("go_to_graph_node.map_frame", "map");
    if (!node->has_parameter("go_to_graph_node.approach_radius"))
        node->declare_parameter<double>("go_to_graph_node.approach_radius", approachRadius);

    timeoutSec = node->get_parameter("go_to_graph_node.timeout").as_double();
    goalTopic  = node->get_parameter("go_to_graph_node.goal_topic").as_string();
    robotFrame = node->get_parameter("go_to_graph_node.robot_frame").as_string();
    mapFrame   = node->get_parameter("go_to_graph_node.map_frame").as_string();
    approachRadius = node->get_parameter("go_to_graph_node.approach_radius").as_double();
}

BT::PortsList GoToGraphNode::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node", "Target GraphNode")
    };
}

BT::NodeStatus GoToGraphNode::onStart()
{
    auto nodeRes = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node");
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

// -------------------- End of navigation.cpp -------------------- //

// Spin::Spin(const std::string &name,
//            const BT::NodeConfiguration &config,
//            rclcpp::Node::SharedPtr nodePtr)
//     : BT::StatefulActionNode(name, config),
//       node(std::move(nodePtr)),
//       robot(std::make_shared<Robot>(node))
// {
//     // Declare ROS parameter for timeout
//     if (!node->has_parameter("spin_node.spin_timeout"))
//         node->declare_parameter<double>("spin_node.spin_timeout", spinTimeout);

//     markerPub = node->create_publisher<visualization_msgs::msg::Marker>(
//         "spin_likeliest_marker", 10);

//     spinTimeout = node->get_parameter("spin_node.spin_timeout").as_double();
// }

// BT::PortsList Spin::providedPorts()
// {
//     return {
//         BT::InputPort<double>("turn_left_angle", 0.0, "CCW rotation angle in radians"),
//         BT::InputPort<double>("turn_right_angle", 0.0, "CW rotation angle in radians"),
//         BT::InputPort<double>("spin_duration", 15.0, "Spin duration seconds"),
//         BT::InputPort<bool>("return_to_likeliest_value", false, "Return to likeliest cosine similarity"),
//         BT::InputPort<std::string>("cosine_similarity_topic", "/value_map/cosine_similarity", "Cosine similarity topic")
//     };
// }

// BT::NodeStatus Spin::onStart()
// {
//     getInput("turn_left_angle",  turnLeftAngle);
//     getInput("turn_right_angle", turnRightAngle);
//     getInput("spin_duration",    spinDuration);
//     getInput("return_to_likeliest_value", returnToLikeliest);
//     getInput("cosine_similarity_topic", cosineTopic);

//     spinTimeout = node->get_parameter("spin_node.spin_timeout").as_double();
//     robot->cancelNavigationGoals();
//     startTimeSteady = steadyClock.now();

//     // --- Store origin yaw in map frame ---
//     geometry_msgs::msg::Pose originPose;
//     if (robot->getPose(originPose, markerFrame, "base_link"))
//         originYaw = tf2::getYaw(originPose.orientation);
//     else
//         originYaw = 0.0;

//     // --- Setup cosine listener if needed ---
//     if (returnToLikeliest)
//     {
//         cosineSamples.clear();
//         cosineSub = node->create_subscription<std_msgs::msg::Float32>(
//             cosineTopic, rclcpp::SensorDataQoS(),
//             std::bind(&Spin::cosineCallback, this, _1));
//         RCLCPP_INFO(node->get_logger(),
//                     "%s[%s] Listening for cosine similarity on '%s'%s",
//                     CYAN, name().c_str(), cosineTopic.c_str(), RESET);
//     }
//     else
//     {
//         cosineSub.reset();
//     }

//     // --- Full turn check ---
//     bool fullTurnNeeded =
//         std::fabs(turnLeftAngle) > 160.0 * M_PI / 180.0 &&
//         std::fabs(turnRightAngle) > 160.0 * M_PI / 180.0;

//     if (fullTurnNeeded)
//     {
//         RCLCPP_INFO(node->get_logger(),
//                     "%s[%s] Large yaw span → full 360° spin%s",
//                     ORANGE, name().c_str(), RESET);
//         robot->spin(2 * M_PI, spinDuration);
//         phase = 99;
//         return BT::NodeStatus::RUNNING;
//     }

//     // --- Build sweep queue ---
//     sweepTargetsAbs.clear();

//     const double ccwAbs = originYaw + std::fabs(turnLeftAngle);
//     const double cwAbs  = originYaw - std::fabs(turnRightAngle);

//     auto push_if_nonzero = [&](double absYaw, double mag)
//     {
//         if (mag > 1e-6) sweepTargetsAbs.push_back(absYaw);
//     };

//     if (std::fabs(turnLeftAngle) < std::fabs(turnRightAngle))
//     {
//         push_if_nonzero(ccwAbs, std::fabs(turnLeftAngle));
//         push_if_nonzero(cwAbs,  std::fabs(turnRightAngle));
//     }
//     else
//     {
//         push_if_nonzero(cwAbs,  std::fabs(turnRightAngle));
//         push_if_nonzero(ccwAbs, std::fabs(turnLeftAngle));
//     }

//     // --- Kick off first sweep ---
//     if (!sweepTargetsAbs.empty())
//     {
//         const double tgt = sweepTargetsAbs.front();
//         RCLCPP_INFO(node->get_logger(),
//                     "%s[%s] Starting sweep to abs yaw %.2f%s",
//                     ORANGE, name().c_str(), tgt, RESET);
//         robot->spinRelativeTo("map", tgt, spinDuration);
//         phase = 1;
//         return BT::NodeStatus::RUNNING;
//     }

//     // --- Nothing to rotate, skip directly to final phase ---
//     phase = 3;
//     return BT::NodeStatus::RUNNING;
// }

// BT::NodeStatus Spin::onRunning()
// {
//     geometry_msgs::msg::Pose currentPose;
//     if (robot->getPose(currentPose, markerFrame, "base_link"))
//         currentYaw = tf2::getYaw(currentPose.orientation);

//     double elapsed = (steadyClock.now() - startTimeSteady).seconds();
//     if (elapsed > spinTimeout)
//     {
//         RCLCPP_WARN(node->get_logger(),
//                     "%s[%s] Timeout after %.2f s → %sFAILURE%s",
//                     RED, name().c_str(), elapsed, RED, RESET);
//         robot->cancelSpin();
//         return BT::NodeStatus::FAILURE;
//     }

//     if (!robot->isSpinDone())
//         return BT::NodeStatus::RUNNING;

//     navResult = robot->getSpinResult();
//     if (navResult != rclcpp_action::ResultCode::SUCCEEDED)
//         return BT::NodeStatus::FAILURE;

//     switch (phase)
//     {
//         // ===================================================
//         // Sweep phase — iterate over queued absolute targets
//         // ===================================================
//         case 1:
//         {
//             if (!sweepTargetsAbs.empty())
//                 sweepTargetsAbs.pop_front();  // remove finished target

//             if (!sweepTargetsAbs.empty())
//             {
//                 const double nextAbs = sweepTargetsAbs.front();

//                 geometry_msgs::msg::Pose pose;
//                 double currentYawNow = currentYaw;
//                 if (robot->getPose(pose, markerFrame, "base_link"))
//                     currentYawNow = tf2::getYaw(pose.orientation);

//                 const double delta = std::atan2(std::sin(nextAbs - currentYawNow),
//                                                 std::cos(nextAbs - currentYawNow));

//                 if (std::fabs(delta) < yaw_epsilon)
//                 {
//                     RCLCPP_INFO(node->get_logger(),
//                                 "%s[%s] Next sweep target %.2f already reached (Δ=%.3f) → skipping%s",
//                                 ORANGE, name().c_str(), nextAbs, delta, RESET);
//                     return BT::NodeStatus::RUNNING;
//                 }

//                 RCLCPP_INFO(node->get_logger(),
//                             "%s[%s] Sweeping to next abs yaw %.2f (Δ=%.3f)%s",
//                             ORANGE, name().c_str(), nextAbs, delta, RESET);

//                 robot->spinRelativeTo("map", nextAbs, spinDuration);
//                 return BT::NodeStatus::RUNNING;
//             }

//             // Queue finished → move to final orientation
//             phase = 3;
//             break;
//         }

//         // ===================================================
//         // Full spin mode (phase 99)
//         // ===================================================
//         case 99:
//             phase = 3;
//             break;

//         // ===================================================
//         // Final orientation (cosine-best or origin)
//         // ===================================================
//         case 3:
//         {
//             double targetAbs = originYaw;

//             if (returnToLikeliest && !cosineSamples.empty())
//             {
//                 targetAbs = findLikeliestYaw();
//                 float bestCos = 0.0f;
//                 for (auto &p : cosineSamples)
//                     if (std::fabs(p.first - targetAbs) < 1e-3)
//                         bestCos = p.second;

//                 publishCosineProfile();
//                 publishDirectionMarker(targetAbs, bestCos);

//                 RCLCPP_INFO(node->get_logger(),
//                             "%s[%s] Orienting to likeliest abs yaw %.2f%s",
//                             CYAN, name().c_str(), targetAbs, RESET);
//             }
//             else
//             {
//                 RCLCPP_INFO(node->get_logger(),
//                             "%s[%s] Returning to origin abs yaw %.2f%s",
//                             ORANGE, name().c_str(), targetAbs, RESET);
//             }

//             robot->spinRelativeTo("map", targetAbs, spinDuration);
//             phase = 4;
//             return BT::NodeStatus::RUNNING;
//         }

//         // ===================================================
//         // Completion
//         // ===================================================
//         case 4:
//         {
//             if (!robot->isSpinDone())
//                 return BT::NodeStatus::RUNNING;

//             RCLCPP_INFO(node->get_logger(),
//                         "%s[%s] Spin sequence complete → %sSUCCESS%s",
//                         GREEN, name().c_str(), GREEN, RESET);
//             return BT::NodeStatus::SUCCESS;
//         }

//         default:
//             return BT::NodeStatus::SUCCESS;
//     }

//     return BT::NodeStatus::RUNNING;
// }

// void Spin::onHalted()
// {
//     robot->cancelSpin();
//     cosineSub.reset();
//     RCLCPP_INFO(node->get_logger(),
//                 "%s[%s] Spin halted.%s",
//                 YELLOW, name().c_str(), RESET);
// }

// double Spin::shortestReturn(double angle)
// {
//     // If already within [-π, π], return the opposite direction
//     if (std::fabs(angle) <= M_PI)
//     {
//         return -angle;
//     }
    
//     // For angles outside [-π, π], wrap to the shortest path back
//     if (angle > 0.0)
//     {
//         return 2.0 * M_PI - angle;  // Positive angle: wrap counter-clockwise
//     }
//     else
//     {
//         return -2.0 * M_PI - angle; // Negative angle: wrap clockwise
//     }
// }

// void Spin::cosineCallback(const std_msgs::msg::Float32::SharedPtr msg)
// {
//     lastCosine = msg->data;
//     cosineSamples.emplace_back(currentYaw, lastCosine);
// }

// double Spin::findLikeliestYaw() const
// {
//     if (cosineSamples.empty())
//         return 0.0;

//     float maxVal = -1.0f;
//     std::vector<double> bestYaws;

//     for (auto &p : cosineSamples)
//     {
//         if (p.second > maxVal + 1e-6)
//         {
//             maxVal = p.second;
//             bestYaws.clear();
//             bestYaws.push_back(p.first);
//         }
//         else if (std::fabs(p.second - maxVal) < 1e-6)
//         {
//             bestYaws.push_back(p.first);
//         }
//     }

//     double sum = 0.0;
//     for (auto &y : bestYaws) sum += y;
//     return sum / bestYaws.size();
// }

// void Spin::publishDirectionMarker(double yaw, float cosineVal)
// {
//     if (!markerPub) return;

//     geometry_msgs::msg::PoseStamped robotPose;
//     if (!robot->getPose(robotPose.pose, "map", "base_link"))
//     {
//         RCLCPP_WARN(node->get_logger(),
//                     "[%s] Could not fetch robot pose for marker publishing", name().c_str());
//         return;
//     }

//     double x = robotPose.pose.position.x;
//     double y = robotPose.pose.position.y;

//     // Arrow length scales with cosine similarity (clamped)
//     double length = std::clamp(static_cast<double>(cosineVal), 0.1, 1.0);

//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = markerFrame;
//     marker.header.stamp = node->now();
//     marker.ns = "spin_likeliest_marker";
//     marker.id = markerId++;
//     marker.type = visualization_msgs::msg::Marker::ARROW;
//     marker.action = visualization_msgs::msg::Marker::ADD;

//     // Arrow position and orientation
//     marker.pose.position.x = x;
//     marker.pose.position.y = y;
//     marker.pose.position.z = 0.1;

//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     marker.pose.orientation = tf2::toMsg(q);

//     marker.scale.x = length;  // arrow length
//     marker.scale.y = 0.05;    // arrow width
//     marker.scale.z = 0.05;    // arrow height

//     marker.color.r = 0.1f;
//     marker.color.g = 1.0f;
//     marker.color.b = 0.2f;
//     marker.color.a = 0.9f;

//     marker.lifetime = rclcpp::Duration::from_seconds(10.0);
//     markerPub->publish(marker);

//     RCLCPP_INFO(node->get_logger(),
//                 "%s[%s] Published marker for highest cosine yaw=%.2f (score=%.3f)%s",
//                 CYAN, name().c_str(), yaw, cosineVal, RESET);
// }

// void Spin::publishCosineProfile()
// {
//     if (cosineSamples.empty() || !markerPub) return;

//     geometry_msgs::msg::PoseStamped robotPose;
//     if (!robot->getPose(robotPose.pose, markerFrame, "base_link"))
//         return;

//     double x0 = robotPose.pose.position.x;
//     double y0 = robotPose.pose.position.y;

//     visualization_msgs::msg::Marker line;
//     line.header.frame_id = markerFrame;
//     line.header.stamp = node->now();
//     line.ns = "spin_cosine_profile";
//     line.id = 999;
//     line.type = visualization_msgs::msg::Marker::LINE_LIST;
//     line.action = visualization_msgs::msg::Marker::ADD;

//     line.scale.x = 0.02;  // line width
//     line.color.a = 1.0;

//     for (auto &p : cosineSamples)
//     {
//         double yaw = p.first;
//         double cosine = std::clamp<double>(p.second, 0.0, 1.0);
//         double length = 0.5 + 0.5 * cosine;  // length proportional to similarity

//         geometry_msgs::msg::Point start, end;
//         start.x = x0;
//         start.y = y0;
//         start.z = 0.05;

//         end.x = x0 + length * std::cos(yaw);
//         end.y = y0 + length * std::sin(yaw);
//         end.z = 0.05;

//         std_msgs::msg::ColorRGBA c;
//         c.r = 1.0f - cosine;
//         c.g = cosine;
//         c.b = 0.1f;
//         c.a = 1.0f;

//         line.points.push_back(start);
//         line.points.push_back(end);
//         line.colors.push_back(c);
//         line.colors.push_back(c);
//     }

//     line.lifetime = rclcpp::Duration::from_seconds(15.0);
//     markerPub->publish(line);

//     RCLCPP_INFO(node->get_logger(),
//                 "%s[%s] Published cosine profile with %zu samples%s",
//                 CYAN, name().c_str(), cosineSamples.size(), RESET);
// }
