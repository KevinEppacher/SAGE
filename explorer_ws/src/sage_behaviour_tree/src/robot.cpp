#include "sage_behaviour_tree/robot.hpp"
#include <tf2/exceptions.h>
#include <cmath>
using namespace std::chrono_literals;

Robot::Robot(rclcpp::Node::SharedPtr nodePtr)
: node(std::move(nodePtr)),
  tfBuffer(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
  tfListener(*tfBuffer)
{
    spinClient = rclcpp_action::create_client<NavSpin>(node, "/spin");
}

bool Robot::getPose(geometry_msgs::msg::Pose& outPose,
                    const std::string& mapFrame,
                    const std::string& robotFrame)
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
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(),
                             2000, "Robot::getPose TF lookup failed: %s", ex.what());
        return false;
    }
}

bool Robot::spin(double yaw, double durationSec)
{
    if (!spinClient)
        spinClient = rclcpp_action::create_client<NavSpin>(node, "/spin");

    if (!spinClient->wait_for_action_server(1s))
    {
        RCLCPP_ERROR(node->get_logger(), "Spin action server not available");
        return false;
    }

    NavSpin::Goal goal;
    goal.target_yaw = yaw;
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
                RCLCPP_INFO(node->get_logger(), "Cancelling active spin goal...");
                spinClient->async_cancel_goal(handle);
                return;
            }
        }

        RCLCPP_INFO(node->get_logger(), "Cancelling all spin goals (no handle yet)...");
        spinClient->async_cancel_all_goals();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(node->get_logger(),
                    "Exception during cancelSpin(): %s", e.what());
    }
    this->halted = true;
    this->haltedTime = node->now();
}

void Robot::cancelNavigationGoals()
{
    auto abortGoal = [&](const std::string& actionName)
    {
        try
        {
            auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, actionName);
            if (client->wait_for_action_server(100ms))
            {
                RCLCPP_INFO(node->get_logger(),
                            "Aborting existing goal on %s", actionName.c_str());
                client->async_cancel_all_goals();
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node->get_logger(),
                        "Exception while aborting %s: %s", actionName.c_str(), e.what());
        }
    };

    abortGoal("/navigate_to_pose");
    abortGoal("/follow_path");
    abortGoal("/compute_path_to_pose");
    this->halted = true;
    this->haltedTime = node->now();
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