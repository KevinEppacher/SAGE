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
        BT::InputPort<double>("spin_angle", 6.28319, "Angle to spin in radians"),
        BT::InputPort<double>("spin_duration", 15.0, "Max spin duration (seconds)")
    };
}

BT::NodeStatus Spin360::onStart()
{
    if (!node_ptr_) return BT::NodeStatus::FAILURE;

    double spin_angle, spin_duration;
    getInput("spin_angle", spin_angle);
    getInput("spin_duration", spin_duration);

    client_ptr_ = rclcpp_action::create_client<Spin>(node_ptr_, "/spin");
    if (!client_ptr_->wait_for_action_server(1s)) return BT::NodeStatus::FAILURE;

    Spin::Goal goal;
    goal.target_yaw = spin_angle;
    goal.time_allowance = rclcpp::Duration::from_seconds(spin_duration);

    using namespace std::placeholders;
    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.result_callback = std::bind(&Spin360::resultCallback, this, _1);

    done_flag_ = false;
    goal_handle_future_ = client_ptr_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Spin360::onRunning()
{
    if (done_flag_)
        return (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
}

void Spin360::onHalted()
{
    if (client_ptr_ && goal_handle_future_.valid()) {
        auto goal_handle = goal_handle_future_.get();
        if (goal_handle) client_ptr_->async_cancel_goal(goal_handle);
    }
}

void Spin360::resultCallback(const GoalHandleSpin::WrappedResult& result)
{
    done_flag_ = true;
    nav_result_ = result.code;
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