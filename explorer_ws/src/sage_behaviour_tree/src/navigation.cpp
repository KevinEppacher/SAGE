#include "sage_behaviour_tree/navigation.hpp"
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

// -------------------- Spin360 -------------------- //

Spin360::Spin360(const std::string& name,
                 const BT::NodeConfiguration& config,
                 rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(std::move(node_ptr))
{}

BT::PortsList Spin360::providedPorts()
{
    return {
        BT::InputPort<double>("min_yaw", 0.0, "Yaw to rotate right (radians, negative)"),
        BT::InputPort<double>("max_yaw", 6.28319, "Yaw to rotate left (radians, positive)"),
        BT::InputPort<double>("spin_duration", 15.0, "Max spin duration (seconds)")
    };
}

BT::NodeStatus Spin360::onStart()
{
    if (!node_ptr_) return BT::NodeStatus::FAILURE;

    getInput("min_yaw", min_yaw_);
    getInput("max_yaw", max_yaw_);
    getInput("spin_duration", spin_duration_);

    client_ptr_ = rclcpp_action::create_client<Spin>(node_ptr_, "/spin");
    if (!client_ptr_->wait_for_action_server(1s))
    {
        RCLCPP_ERROR(node_ptr_->get_logger(),
                     "[%s] Spin action server not available.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    done_flag_ = false;
    phase_two_ = false;

    // Start with first spin direction
    if (max_yaw_ != 0.0)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Starting first spin: +%.2f rad", name().c_str(), max_yaw_);
        sendSpinGoal(max_yaw_, spin_duration_);
    }
    else if (min_yaw_ != 0.0)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Only min_yaw specified: -%.2f rad", name().c_str(), std::abs(min_yaw_));
        sendSpinGoal(min_yaw_, spin_duration_);
    }
    else
    {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[%s] Both yaw limits are 0 — skipping rotation.", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Spin360::onRunning()
{
    if (!done_flag_) return BT::NodeStatus::RUNNING;

    // First spin completed, start second one if needed
    if (!phase_two_ && min_yaw_ != 0.0 && max_yaw_ != 0.0)
    {
        phase_two_ = true;
        done_flag_ = false;

        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] First spin done — now spinning back %.2f rad", name().c_str(), min_yaw_);
        sendSpinGoal(min_yaw_, spin_duration_);
        return BT::NodeStatus::RUNNING;
    }

    // All done
    if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Spin complete.", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node_ptr_->get_logger(),
                "[%s] Spin failed or cancelled (code %d).",
                name().c_str(), static_cast<int>(nav_result_));
    return BT::NodeStatus::FAILURE;
}

void Spin360::onHalted()
{
    if (client_ptr_ && goal_handle_future_.valid())
    {
        auto goal_handle = goal_handle_future_.get();
        if (goal_handle)
        {
            RCLCPP_INFO(node_ptr_->get_logger(),
                        "[%s] Cancelling spin...", name().c_str());
            client_ptr_->async_cancel_goal(goal_handle);
        }
    }
}

void Spin360::sendSpinGoal(double yaw, double duration)
{
    Spin::Goal goal;
    goal.target_yaw = yaw;
    goal.time_allowance = rclcpp::Duration::from_seconds(duration);

    using namespace std::placeholders;
    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.result_callback = std::bind(&Spin360::resultCallback, this, _1);

    goal_handle_future_ = client_ptr_->async_send_goal(goal, options);
}

void Spin360::resultCallback(const GoalHandleSpin::WrappedResult& result)
{
    done_flag_ = true;
    nav_result_ = result.code;

    std::string code;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED: code = "SUCCEEDED"; break;
        case rclcpp_action::ResultCode::ABORTED: code = "ABORTED"; break;
        case rclcpp_action::ResultCode::CANCELED: code = "CANCELED"; break;
        default: code = "UNKNOWN"; break;
    }

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Spin result: %s", name().c_str(), code.c_str());
}

// -------------------- GoToGraphNode -------------------- //

GoToGraphNode::GoToGraphNode(const std::string& name,
                             const BT::NodeConfiguration& config,
                             rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config),
      node_ptr_(std::move(node_ptr))
{}

BT::PortsList GoToGraphNode::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "graph_nodes", "GraphNode to navigate to"),
        BT::InputPort<double>("approach_radius", 2.0, "Approach distance in meters"),
        BT::InputPort<std::string>("goal_topic", "/goal_pose", "Goal topic to publish to")
    };
}

BT::NodeStatus GoToGraphNode::onStart()
{
    if (!node_ptr_) {
        RCLCPP_ERROR(rclcpp::get_logger("GoToGraphNode"),
                     "[%s] Node pointer is null!", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    getInput("approach_radius", approach_radius_);
    getInput("goal_topic", goal_topic_);

    auto node_res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes");
    if (!node_res) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[%s] No graph_nodes input provided.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    target_node_ = node_res.value();

    // Create publisher if not yet created
    if (!goal_pub_) {
        goal_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Publishing goals to topic: %s",
                    name().c_str(), goal_topic_.c_str());
    }

    last_publish_time_ = node_ptr_->now();
    publishGoalToTarget(*target_node_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToGraphNode::onRunning()
{
    // Check if new goal arrived on blackboard
    auto node_res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes");
    if (node_res && targetChanged(*node_res.value()))
        target_node_ = node_res.value();

    // Publish periodically
    auto now = node_ptr_->now();
    if ((now - last_publish_time_).seconds() >= 0.1) {
        publishGoalToTarget(*target_node_);
        last_publish_time_ = now;
    }

    return BT::NodeStatus::RUNNING;
}

void GoToGraphNode::onHalted()
{
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Halting — stopping goal publishing.", name().c_str());
}

void GoToGraphNode::publishGoalToTarget(const graph_node_msgs::msg::GraphNode& node)
{
    double rx = 0.0, ry = 0.0;  // TODO: Replace with TF lookup for robot pose
    double tx = node.position.x;
    double ty = node.position.y;

    double dx = tx - rx;
    double dy = ty - ry;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-3) return;

    double scale = (dist - approach_radius_) / dist;
    geometry_msgs::msg::Point approach_point;
    approach_point.x = rx + dx * scale;
    approach_point.y = ry + dy * scale;
    approach_point.z = node.position.z;

    double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_ptr_->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position = approach_point;
    goal_msg.pose.orientation = orientation;

    goal_pub_->publish(goal_msg);

    RCLCPP_DEBUG(node_ptr_->get_logger(),
                 "[%s] Published goal (%.2f, %.2f), yaw=%.2f rad",
                 name().c_str(), approach_point.x, approach_point.y, yaw);
}

bool GoToGraphNode::targetChanged(const graph_node_msgs::msg::GraphNode& new_target) const
{
    if (!target_node_) return true;
    if (target_node_->id != new_target.id) return true;

    double dx = std::abs(target_node_->position.x - new_target.position.x);
    double dy = std::abs(target_node_->position.y - new_target.position.y);
    return (dx > 0.2 || dy > 0.2);
}