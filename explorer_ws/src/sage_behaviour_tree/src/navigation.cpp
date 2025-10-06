#include "sage_behaviour_tree/navigation.hpp"
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    : BT::StatefulActionNode(name, config), node_ptr_(std::move(node_ptr))
{}

BT::PortsList GoToGraphNode::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "graph_nodes", "GraphNode to navigate to")
    };
}

BT::NodeStatus GoToGraphNode::onStart()
{
    if (!node_ptr_) return BT::NodeStatus::FAILURE;

    auto node_res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes");
    if (!node_res) return BT::NodeStatus::FAILURE;

    target_node_ = node_res.value();
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    if (!client_ptr_->wait_for_action_server(1s)) return BT::NodeStatus::FAILURE;

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_ptr_->now();
    goal.pose.pose.position = target_node_->position;
    goal.pose.pose.orientation.w = 1.0;

    using namespace std::placeholders;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback = std::bind(&GoToGraphNode::resultCallback, this, _1);

    done_flag_ = false;
    goal_handle_future_ = client_ptr_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToGraphNode::onRunning()
{
    if (done_flag_) {
        if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED)
            return BT::NodeStatus::SUCCESS;
        else
            return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToGraphNode::onHalted()
{
    if (client_ptr_ && goal_handle_future_.valid()) {
        auto goal_handle = goal_handle_future_.get();
        if (goal_handle) client_ptr_->async_cancel_goal(goal_handle);
    }
}

void GoToGraphNode::resultCallback(const GoalHandleNav::WrappedResult& result)
{
    done_flag_ = true;
    nav_result_ = result.code;
}
