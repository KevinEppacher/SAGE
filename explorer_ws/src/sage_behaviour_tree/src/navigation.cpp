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
    // Declare ROS parameter for timeout
    if (!node->has_parameter("spin_node.spin_timeout"))
        node->declare_parameter<double>("spin_node.spin_timeout", spinTimeout);

    markerPub = node->create_publisher<visualization_msgs::msg::Marker>(
        "spin_likeliest_marker", 10);

    spinTimeout = node->get_parameter("spin_node.spin_timeout").as_double();
}

BT::PortsList Spin::providedPorts()
{
    return {
        BT::InputPort<double>("turn_left_angle", 0.0, "CCW rotation angle in radians"),
        BT::InputPort<double>("turn_right_angle", 0.0, "CW rotation angle in radians"),
        BT::InputPort<double>("spin_duration", 15.0, "Spin duration seconds"),
        BT::InputPort<bool>("return_to_likeliest_value", false, "Return to likeliest cosine similarity"),
        BT::InputPort<std::string>("cosine_similarity_topic", "/value_map/cosine_similarity", "Cosine similarity topic")
    };
}

BT::NodeStatus Spin::onStart()
{
    getInput("turn_left_angle",  turnLeftAngle);
    getInput("turn_right_angle", turnRightAngle);
    getInput("spin_duration",    spinDuration);
    getInput("return_to_likeliest_value", returnToLikeliest);
    getInput("cosine_similarity_topic", cosineTopic);

    spinTimeout = node->get_parameter("spin_node.spin_timeout").as_double();
    robot->cancelNavigationGoals();
    startTimeSteady = steadyClock.now();

    // Store origin pose yaw (in map frame)
    geometry_msgs::msg::Pose originPose;
    if (robot->getPose(originPose, markerFrame, "base_link"))
        originYaw = tf2::getYaw(originPose.orientation);
    else
        originYaw = 0.0;

    // Setup cosine listener if needed
    if (returnToLikeliest)
    {
        cosineSamples.clear();
        cosineSub = node->create_subscription<std_msgs::msg::Float32>(
            cosineTopic, rclcpp::SensorDataQoS(),
            std::bind(&Spin::cosineCallback, this, _1));
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Listening for cosine similarity on '%s'%s",
                    CYAN, name().c_str(), cosineTopic.c_str(), RESET);
    }
    else
    {
        cosineSub.reset();
    }

    // --- phase selection ---
    bool fullTurnNeeded =
        std::fabs(turnLeftAngle) > 120.0 * M_PI / 180.0 &&
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

    if (turnLeftAngle != 0.0)
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Phase 1: spinning CCW %.2f rad%s",
                    ORANGE, name().c_str(), turnLeftAngle, RESET);
        robot->spin(turnLeftAngle, spinDuration);
        phase = 1;
        return BT::NodeStatus::RUNNING;
    }

    if (turnRightAngle != 0.0)
    {
        RCLCPP_INFO(node->get_logger(),
                    "%s[%s] Phase 2: spinning CW %.2f rad%s",
                    ORANGE, name().c_str(), turnRightAngle, RESET);
        robot->spin(turnRightAngle, spinDuration);
        phase = 2;
        return BT::NodeStatus::RUNNING;
    }

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] No rotation requested → %sSUCCESS%s",
                GREEN, name().c_str(), GREEN, RESET);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Spin::onRunning()
{
    geometry_msgs::msg::Pose currentPose;
    if (robot->getPose(currentPose, markerFrame, "base_link"))
        currentYaw = tf2::getYaw(currentPose.orientation);

    double elapsed = (steadyClock.now() - startTimeSteady).seconds();
    if (elapsed > spinTimeout)
    {
        RCLCPP_WARN(node->get_logger(),
                    "%s[%s] Timeout after %.2f s → %sFAILURE%s",
                    RED, name().c_str(), elapsed, RED, RESET);
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
            // CCW done → spin CW if requested
            if (turnRightAngle != 0.0)
            {
                double cwSpan = -(turnLeftAngle + std::fabs(turnRightAngle));
                RCLCPP_INFO(node->get_logger(),
                            "%s[%s] Phase 2: sweeping CW %.2f rad%s",
                            ORANGE, name().c_str(), cwSpan, RESET);
                robot->spin(cwSpan, spinDuration);
                phase = 2;
                return BT::NodeStatus::RUNNING;
            }
            phase = 3;
            break;
        }

        case 2:
        case 99:
            // Both partial and full spins end here
            phase = 3;
            break;

        case 3:
        {
            // --- Final orientation phase ---
            double diffYaw = 0.0;
            if (returnToLikeliest && !cosineSamples.empty())
            {
                double bestYaw = findLikeliestYaw();
                // diffYaw = shortestReturn(bestYaw - currentYaw);
                diffYaw = bestYaw - currentYaw;

                float bestCosine = 0.0f;
                for (auto &p : cosineSamples)
                    if (std::fabs(p.first - bestYaw) < 1e-3)
                        bestCosine = p.second;

                publishCosineProfile();
                publishDirectionMarker(bestYaw, bestCosine);

                RCLCPP_INFO(node->get_logger(),
                            "%s[%s] Orienting to likeliest yaw: target %.2f (Δ=%.2f)%s",
                            CYAN, name().c_str(), bestYaw, diffYaw, RESET);
            }
            else
            {
                diffYaw = shortestReturn(originYaw - currentYaw);
                RCLCPP_INFO(node->get_logger(),
                            "%s[%s] Returning to origin (Δ=%.2f rad)%s",
                            ORANGE, name().c_str(), diffYaw, RESET);
            }

            robot->spin(diffYaw, spinDuration);
            phase = 4;
            return BT::NodeStatus::RUNNING;
        }

        case 4:
        {
            if (!robot->isSpinDone())
                return BT::NodeStatus::RUNNING;

            RCLCPP_INFO(node->get_logger(),
                        "%s[%s] Spin sequence complete → %sSUCCESS%s",
                        GREEN, name().c_str(), GREEN, RESET);
            return BT::NodeStatus::SUCCESS;
        }

        default:
            return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void Spin::onHalted()
{
    robot->cancelSpin();
    cosineSub.reset();
    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Spin halted.%s",
                YELLOW, name().c_str(), RESET);
}

double Spin::shortestReturn(double angle)
{
    // If already within [-π, π], return the opposite direction
    if (std::fabs(angle) <= M_PI)
    {
        return -angle;
    }
    
    // For angles outside [-π, π], wrap to the shortest path back
    if (angle > 0.0)
    {
        return 2.0 * M_PI - angle;  // Positive angle: wrap counter-clockwise
    }
    else
    {
        return -2.0 * M_PI - angle; // Negative angle: wrap clockwise
    }
}

void Spin::cosineCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    lastCosine = msg->data;
    cosineSamples.emplace_back(currentYaw, lastCosine);
}

double Spin::findLikeliestYaw() const
{
    if (cosineSamples.empty())
        return 0.0;

    float maxVal = -1.0f;
    std::vector<double> bestYaws;

    for (auto &p : cosineSamples)
    {
        if (p.second > maxVal + 1e-6)
        {
            maxVal = p.second;
            bestYaws.clear();
            bestYaws.push_back(p.first);
        }
        else if (std::fabs(p.second - maxVal) < 1e-6)
        {
            bestYaws.push_back(p.first);
        }
    }

    double sum = 0.0;
    for (auto &y : bestYaws) sum += y;
    return sum / bestYaws.size();
}

void Spin::publishDirectionMarker(double yaw, float cosineVal)
{
    if (!markerPub) return;

    geometry_msgs::msg::PoseStamped robotPose;
    if (!robot->getPose(robotPose.pose, "map", "base_link"))
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s] Could not fetch robot pose for marker publishing", name().c_str());
        return;
    }

    double x = robotPose.pose.position.x;
    double y = robotPose.pose.position.y;

    // Arrow length scales with cosine similarity (clamped)
    double length = std::clamp(static_cast<double>(cosineVal), 0.1, 1.0);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = markerFrame;
    marker.header.stamp = node->now();
    marker.ns = "spin_likeliest_marker";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Arrow position and orientation
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.1;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = length;  // arrow length
    marker.scale.y = 0.05;    // arrow width
    marker.scale.z = 0.05;    // arrow height

    marker.color.r = 0.1f;
    marker.color.g = 1.0f;
    marker.color.b = 0.2f;
    marker.color.a = 0.9f;

    marker.lifetime = rclcpp::Duration::from_seconds(10.0);
    markerPub->publish(marker);

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Published marker for highest cosine yaw=%.2f (score=%.3f)%s",
                CYAN, name().c_str(), yaw, cosineVal, RESET);
}

void Spin::publishCosineProfile()
{
    if (cosineSamples.empty() || !markerPub) return;

    geometry_msgs::msg::PoseStamped robotPose;
    if (!robot->getPose(robotPose.pose, markerFrame, "base_link"))
        return;

    double x0 = robotPose.pose.position.x;
    double y0 = robotPose.pose.position.y;

    visualization_msgs::msg::Marker line;
    line.header.frame_id = markerFrame;
    line.header.stamp = node->now();
    line.ns = "spin_cosine_profile";
    line.id = 999;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;

    line.scale.x = 0.02;  // line width
    line.color.a = 1.0;

    for (auto &p : cosineSamples)
    {
        double yaw = p.first;
        double cosine = std::clamp<double>(p.second, 0.0, 1.0);
        double length = 0.5 + 0.5 * cosine;  // length proportional to similarity

        geometry_msgs::msg::Point start, end;
        start.x = x0;
        start.y = y0;
        start.z = 0.05;

        end.x = x0 + length * std::cos(yaw);
        end.y = y0 + length * std::sin(yaw);
        end.z = 0.05;

        std_msgs::msg::ColorRGBA c;
        c.r = 1.0f - cosine;
        c.g = cosine;
        c.b = 0.1f;
        c.a = 1.0f;

        line.points.push_back(start);
        line.points.push_back(end);
        line.colors.push_back(c);
        line.colors.push_back(c);
    }

    line.lifetime = rclcpp::Duration::from_seconds(15.0);
    markerPub->publish(line);

    RCLCPP_INFO(node->get_logger(),
                "%s[%s] Published cosine profile with %zu samples%s",
                CYAN, name().c_str(), cosineSamples.size(), RESET);
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
