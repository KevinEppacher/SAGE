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

    // ---- Abort all other navigation actions ---- //
    {
        auto abort_nav_goal = [&](const std::string& action_name)
        {
            try
            {
                auto generic_client =
                    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                        node_ptr_, action_name);
                if (generic_client->wait_for_action_server(100ms))
                {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                                "[%s] Aborting existing goal on %s", name().c_str(),
                                action_name.c_str());
                    generic_client->async_cancel_all_goals();
                }
            }
            catch (...) {}
        };

        abort_nav_goal("/navigate_to_pose");
        abort_nav_goal("/follow_path");
        abort_nav_goal("/compute_path_to_pose");
    }

    done_flag_ = false;
    phase_two_ = false;

    // ---- Start spin ---- //
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
    if (was_halted_) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Halted externally — skipping further spins.", name().c_str());
        was_halted_ = false;      // reset for next run
        return BT::NodeStatus::FAILURE;
    }

    if (!done_flag_) return BT::NodeStatus::RUNNING;

    if (!phase_two_ && min_yaw_ != 0.0 && max_yaw_ != 0.0)
    {
        phase_two_ = true;
        done_flag_ = false;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] First spin done — now spinning back %.2f rad", name().c_str(), min_yaw_);
        sendSpinGoal(min_yaw_, spin_duration_);
        return BT::NodeStatus::RUNNING;
    }

    if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Spin complete.", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node_ptr_->get_logger(),
                "[%s] Spin failed or cancelled (code %d).",
                name().c_str(), static_cast<int>(nav_result_));
    return BT::NodeStatus::FAILURE;
}

void Spin360::onHalted()
{
    was_halted_ = true;

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
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// -------------------- Ports -------------------- //
BT::PortsList GoToGraphNode::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "graph_nodes", "GraphNode to navigate to"),
        BT::InputPort<double>("approach_radius", 2.0, "Approach distance in meters"),
        BT::InputPort<std::string>("goal_topic", "/goal_pose", "Goal topic to publish to"),
        BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame for TF lookup"),
        BT::InputPort<int>("max_changed_targets", 3, "Maximum number of changed targets"),
        BT::InputPort<std::string>("map_frame", "map", "Map frame for TF lookup")
    };
}

// -------------------- onStart -------------------- //
BT::NodeStatus GoToGraphNode::onStart()
{
    if (!node_ptr_) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "[%s] Node pointer is null!", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    getInput("approach_radius", approach_radius_);
    getInput("goal_topic", goal_topic_);
    getInput("robot_frame", robot_frame_);
    getInput("map_frame", map_frame_);
    getInput("max_changed_targets", max_changed_targets_);

    changed_target_count_ = 0;  // reset counter for new execution

    auto node_res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes");
    if (!node_res) {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] No graph_nodes input provided.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    target_node_ = node_res.value();

    if (!goal_pub_) {
        goal_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Publishing goals to topic: %s", name().c_str(), goal_topic_.c_str());
    }

    // Check if already close enough
    if (isWithinGoal(*target_node_)) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Already within approach radius (%.2fm) → SUCCESS.",
                    name().c_str(), approach_radius_);
        return BT::NodeStatus::SUCCESS;
    }

    publishGoalToTarget(*target_node_);
    return BT::NodeStatus::RUNNING;
}

// -------------------- onRunning -------------------- //
BT::NodeStatus GoToGraphNode::onRunning()
{
    if (!target_node_) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[%s] No active target node. Returning FAILURE.", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Check proximity before publishing
    if (isWithinGoal(*target_node_)) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Within %.2fm of target — SUCCESS.",
                    name().c_str(), approach_radius_);
        return BT::NodeStatus::SUCCESS;
    }

    auto node_res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes");

    if (node_res && targetChanged(*node_res.value()) && changed_target_count_ < max_changed_targets_) {
        changed_target_count_++;
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Target changed, updating goal (change count: %d).",
                    name().c_str(), changed_target_count_);
        target_node_ = node_res.value();
        publishGoalToTarget(*target_node_);
    }

    if(isWithinGoal(*target_node_)) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Within %.2fm of target after update — SUCCESS.",
                    name().c_str(), approach_radius_);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

// -------------------- onHalted -------------------- //
void GoToGraphNode::onHalted()
{
    changed_target_count_ = 0;

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Halting — stopping goal publishing.", name().c_str());
}

// -------------------- isWithinGoal -------------------- //
bool GoToGraphNode::isWithinGoal(const graph_node_msgs::msg::GraphNode& node)
{
    try {
        geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);

        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;
        double tx = node.position.x;
        double ty = node.position.y;

        double dist = std::hypot(tx - rx, ty - ry);
        return dist <= approach_radius_;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(),
                             2000, "[%s] TF lookup failed: %s", name().c_str(), ex.what());
        return false;
    }
}

// -------------------- publishGoalToTarget -------------------- //
void GoToGraphNode::publishGoalToTarget(const graph_node_msgs::msg::GraphNode& node)
{
    double tx = node.position.x;
    double ty = node.position.y;

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_ptr_->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = tx;
    goal_msg.pose.position.y = ty;
    goal_msg.pose.position.z = node.position.z;

    // Orientation toward goal
    goal_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
    goal_pub_->publish(goal_msg);

    RCLCPP_DEBUG(node_ptr_->get_logger(),
                 "[%s] Published goal (%.2f, %.2f)",
                 name().c_str(), tx, ty);
}

// -------------------- targetChanged -------------------- //
bool GoToGraphNode::targetChanged(const graph_node_msgs::msg::GraphNode& new_target) const
{
    if (!target_node_) return true;
    if (target_node_->id != new_target.id) return true;

    double dx = std::abs(target_node_->position.x - new_target.position.x);
    double dy = std::abs(target_node_->position.y - new_target.position.y);
    return (dx > 0.2 || dy > 0.2);
}