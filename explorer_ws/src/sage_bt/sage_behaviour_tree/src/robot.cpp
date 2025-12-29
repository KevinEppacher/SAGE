#include "sage_behaviour_tree/robot.hpp"
#include "sage_behaviour_tree/colors.hpp"
#include <tf2/exceptions.h>
#include <cmath>
using namespace std::chrono_literals;

static inline double normalizeAngle(double a)
{
    // Wrap to [-pi, pi]
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static inline double shortestAngularDistance(double from, double to)
{
    // Smallest signed angle to rotate from -> to
    return normalizeAngle(to - from);
}

Robot::Robot(rclcpp::Node::SharedPtr nodePtr)
: node(std::move(nodePtr)),
  tfBuffer(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
  tfListener(*tfBuffer)
{
    spinClient = rclcpp_action::create_client<NavSpin>(node, "/spin");
    navClient = rclcpp_action::create_client<NavigateToPose>(node, "/navigate_to_pose");

    // Subscribe to Nav2 costmaps
    costmapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&Robot::costmapCallback, this, std::placeholders::_1));

    mapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&Robot::mapCallback, this, std::placeholders::_1));
}

bool Robot::getPose(geometry_msgs::msg::Pose& outPose,
                    const std::string& mapFrame,
                    const std::string& robotFrame)
{
    const int maxRetries = 5;
    const double baseDelay = 0.1;  // seconds
    const double maxWait = 1.0;    // seconds total

    // Use a steady, monotonic clock for measuring timeouts
    rclcpp::Clock steadyClock(RCL_STEADY_TIME);
    auto start = steadyClock.now();

    for (int attempt = 0; attempt < maxRetries; ++attempt)
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tfBuffer->lookupTransform(mapFrame, robotFrame, tf2::TimePointZero);

            outPose.position.x = tf.transform.translation.x;
            outPose.position.y = tf.transform.translation.y;
            outPose.position.z = tf.transform.translation.z;
            outPose.orientation = tf.transform.rotation;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            auto elapsed = (steadyClock.now() - start).seconds();
            if (elapsed > maxWait)
            {
                RCLCPP_WARN(node->get_logger(),
                            "[%s][Robot] Robot::getPose() - Failed after %.2f s: %s",
                            currentContext.c_str(), elapsed, ex.what());
                return false;
            }

            double delay = baseDelay * std::pow(1.5, attempt);
            RCLCPP_WARN(node->get_logger(),
                        "[%s][Robot] Robot::getPose() - TF lookup failed (try %d/%d): %s. Retrying in %.2f s...",
                        currentContext.c_str(), attempt + 1, maxRetries, ex.what(), delay);

            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(delay)));
        }
    }

    RCLCPP_ERROR(node->get_logger(),
                 "[%s][Robot] Robot::getPose() - Failed to get transform after %d retries.", currentContext.c_str(), maxRetries);
    return false;
}

bool Robot::spin(double yaw, double durationSec)
{
    if (!spinClient)
        spinClient = rclcpp_action::create_client<NavSpin>(node, "/spin");

    if (!spinClient->wait_for_action_server(1s))
    {
        RCLCPP_ERROR(node->get_logger(), "[%s][Robot] Spin action server not available", currentContext.c_str());
        return false;
    }

    NavSpin::Goal goal;
    goal.target_yaw = yaw;
    goal.disable_collision_checks = true;
    goal.time_allowance = rclcpp::Duration::from_seconds(durationSec);

    spinDone = false;
    spinResult = rclcpp_action::ResultCode::UNKNOWN;

    auto options = rclcpp_action::Client<NavSpin>::SendGoalOptions();
    options.result_callback = [this](const GoalHandleSpin::WrappedResult& result)
    {
        spinDone = true;
        spinResult = result.code;
    };

    goalHandleFuture = spinClient->async_send_goal(goal, options);
    this->halted = false;
    return true;
}

bool Robot::isSpinDone() const
{
    return spinDone;
}

rclcpp_action::ResultCode Robot::getSpinResult() const
{
    return spinResult;
}

void Robot::cancelSpin()
{
    if (!spinClient)
        return;

    try
    {
        if (goalHandleFuture.valid() &&
            goalHandleFuture.wait_for(0s) == std::future_status::ready)
        {
            auto handle = goalHandleFuture.get();
            if (handle)
            {
                RCLCPP_INFO(node->get_logger(), "[%s][Robot] Cancelling active spin goal...", currentContext.c_str());
                spinClient->async_cancel_goal(handle);
                return;
            }
        }

        RCLCPP_INFO(node->get_logger(), "[%s][Robot] Cancelling all spin goals (no handle yet)...", currentContext.c_str());
        spinClient->async_cancel_all_goals();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s][Robot] Exception during cancelSpin(): %s", currentContext.c_str(), e.what());
    }
    spinDone = true;
    spinResult = rclcpp_action::ResultCode::CANCELED;
    halted = true;
    haltedTime = node->now();
}

void Robot::publishGoalToTarget(const graph_node_msgs::msg::GraphNode& nodeMsg,
                                const std::string& goalTopic,
                                const std::string& frame)
{
    auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(goalTopic, 10);
    geometry_msgs::msg::PoseStamped msg;

    msg.header.frame_id = frame;
    msg.header.stamp = node->now();
    msg.pose.position.x = nodeMsg.position.x;
    msg.pose.position.y = nodeMsg.position.y;
    msg.pose.position.z = nodeMsg.position.z;
    msg.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

    pub->publish(msg);
    this->halted = false;
}

bool Robot::isHalted()
{
    // True only for 1 second after a halt
    return this->halted &&
           (node->now() - this->haltedTime) < rclcpp::Duration::from_seconds(1.0);
}

std::shared_ptr<nav_msgs::msg::OccupancyGrid> Robot::getGlobalCostmap()
{
    if (!latestCostmap)
    {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "[%s][Robot] Robot::getGlobalCostmap() → No costmap received yet.", currentContext.c_str());
        return nullptr;
    }
    return latestCostmap;
}

void Robot::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latestCostmap = msg;
}

void Robot::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latestMap = msg;
}

bool Robot::spinRelativeTo(const std::string& frame, double targetYawAbs, double durationSec)
{
    // 1) Get current yaw in 'frame'
    geometry_msgs::msg::Pose pose;
    if (!getPose(pose, frame, "base_link"))
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s][Robot] spinRelativeTo(): can't get pose in frame '%s'", currentContext.c_str(), frame.c_str());
        return false;
    }
    const double currentYaw = tf2::getYaw(pose.orientation);

    // 2) Compute minimal relative rotation to reach absolute target
    const double delta = shortestAngularDistance(currentYaw, targetYawAbs);

    if (std::fabs(delta) < 0.01) {   // ~0.6 deg
        RCLCPP_INFO(node->get_logger(),
                    "[%s][Robot] spinRelativeTo('%s'): already aligned (|Δ|=%.3f) → no-op",
                    currentContext.c_str(), frame.c_str(), delta);
        // Mimic immediate success: do NOT set any internal flags here; just return true.
        return true;
    }

    return spin(delta, durationSec);
}

bool Robot::navigateToPose(const geometry_msgs::msg::Pose& goal, const std::string& frame)
{
    if (!navClient)
        navClient = rclcpp_action::create_client<NavigateToPose>(node, "/navigate_to_pose");

    if (!navClient->wait_for_action_server(2s))
    {
        RCLCPP_ERROR(node->get_logger(), "[%s][Robot] Nav2 NavigateToPose server not available", currentContext.c_str());
        return false;
    }

    NavigateToPose::Goal navGoal;
    navGoal.pose.header.frame_id = frame;
    navGoal.pose.header.stamp = node->now();
    navGoal.pose.pose = goal;

    auto goalFuture = navClient->async_send_goal(navGoal);
    if (goalFuture.wait_for(1s) != std::future_status::ready)
        return false;

    currentGoalHandle = goalFuture.get();
    if (!currentGoalHandle)
        return false;

    navResultFuture = navClient->async_get_result(currentGoalHandle);
    navigating = true;
    navSucceeded = false;
    navStart = node->now();
    return true;
}

bool Robot::isNavigationRunning() const
{
    return navigating && !navResultFuture.valid() ? false :
           navigating && navResultFuture.wait_for(0s) != std::future_status::ready;
}

bool Robot::navigationSucceeded()
{
    if (!navigating) return false;
    if (navResultFuture.wait_for(0s) != std::future_status::ready)
        return false;

    auto result = navResultFuture.get();
    navSucceeded = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    navigating = false;
    return navSucceeded;
}

void Robot::cancelNavigation()
{
    if (!navClient)
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s][Robot] cancelNavigation(): navClient not initialized.", currentContext.c_str());
        return;
    }

    // Prevent redundant cancels
    if (!navigating)
    {
        RCLCPP_DEBUG(node->get_logger(),
                     "[%s][Robot] cancelNavigation(): no active navigation in progress.", currentContext.c_str());
        return;
    }

    try
    {
        if (currentGoalHandle)
        {
            auto status = currentGoalHandle->get_status();

            if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                status == rclcpp_action::GoalStatus::STATUS_EXECUTING)
            {
                RCLCPP_INFO(node->get_logger(),
                            "[%s][Robot] Cancelling Nav2 navigation goal (status %d)...", currentContext.c_str(), status);
                navClient->async_cancel_goal(currentGoalHandle);
            }
            else
            {
                RCLCPP_DEBUG(node->get_logger(),
                             "[%s][Robot] Goal handle exists but status=%d (not active) — skipping cancel.",
                             currentContext.c_str(), status);
            }
        }
        else
        {
            RCLCPP_INFO(node->get_logger(),
                        "[%s][Robot] No goal handle found — cancelling all Nav2 goals instead...", currentContext.c_str());
            navClient->async_cancel_all_goals();
        }
    }
    catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s][Robot] cancelNavigation(): UnknownGoalHandleError ignored — %s",
                    currentContext.c_str(), e.what());
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(node->get_logger(),
                    "[%s][Robot] cancelNavigation(): exception while cancelling: %s", currentContext.c_str(), e.what());
    }

    navigating = false;
    navSucceeded = false;
    currentGoalHandle.reset();
    navResultFuture = std::shared_future<typename GoalHandleNav::WrappedResult>();
    halted = true;
    haltedTime = node->now();
}

void Robot::recordCurrentPose()
{
    geometry_msgs::msg::Pose current;
    if (!getPose(current)) return;

    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = "map";
    stamped.header.stamp = node->now();
    stamped.pose = current;

    // store start pose once
    if (!startPoseSet)
    {
        startPose = current;
        startPoseSet = true;
    }

    pathHistory.push_back(stamped);
    endPose = current;
}

nav_msgs::msg::Path Robot::getAccumulatedPath(const std::string& reference_frame_id) const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = reference_frame_id;
    path.header.stamp = node->now();
    path.poses = pathHistory;
    return path;
}

void Robot::resetPath()
{
    pathHistory.clear();
    startPoseSet = false;
}