#include "sage_behaviour_tree/navigation.hpp"
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

// -------------------- Spin -------------------- //

Spin::Spin(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config),
      node_ptr_(std::move(node_ptr)) {}

BT::PortsList Spin::providedPorts()
{
  return {
      BT::InputPort<double>("min_yaw", 0.0, "CW yaw (negative radians)"),
      BT::InputPort<double>("max_yaw", 0.0, "CCW yaw (positive radians)"),
      BT::InputPort<double>("spin_duration", 15.0, "Spin duration seconds")};
}

static double shortestReturn(double angle)
{
  // Minimal rotation back to 0 radians
  if (std::fabs(angle) <= M_PI)
    return -angle;

  // Adjust direction to take shorter path
  if (angle > 0.0)
    return 2 * M_PI - angle;
  else
    return -2 * M_PI - angle;
}

BT::NodeStatus Spin::onStart()
{
  getInput("min_yaw", min_yaw_);
  getInput("max_yaw", max_yaw_);
  getInput("spin_duration", spin_duration_);

  client_ptr_ = rclcpp_action::create_client<NavSpin>(node_ptr_, "/spin");
  if (!client_ptr_->wait_for_action_server(1s))
  {
    RCLCPP_ERROR(node_ptr_->get_logger(),
                 "[%s] Spin action server not available", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  done_flag_ = false;
  phase_ = 0;
  cumulative_rotation_ = 0.0;

  if (max_yaw_ == 0.0 && min_yaw_ == 0.0)
  {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "[%s] Both yaw values are 0 → nothing to spin.", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // ---- Phase 1: Spin CCW to max_yaw ----
  if (max_yaw_ != 0.0)
  {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Phase 1: spinning CCW to %.2f rad", name().c_str(), max_yaw_);
    sendSpinGoal(max_yaw_);
    phase_ = 1;
  }
  else
  {
    // Skip directly to CW sequence
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Skipping CCW phase (max_yaw = 0).", name().c_str());
    if (min_yaw_ != 0.0)
    {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Phase 3: spinning CW to %.2f rad", name().c_str(), min_yaw_);
      sendSpinGoal(min_yaw_);
      phase_ = 3;
    }
    else
    {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Spin::onRunning()
{
  if (!done_flag_)
    return BT::NodeStatus::RUNNING;

  done_flag_ = false;

  if (nav_result_ != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "[%s] Spin failed or canceled.", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  switch (phase_)
  {
  case 1:
  {
    // ---- Phase 2: Return to 0 from CCW ----
    double return_yaw = shortestReturn(max_yaw_);
    cumulative_rotation_ += std::fabs(max_yaw_);
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Phase 2: returning to 0 rad (shortest path %.2f rad)",
                name().c_str(), return_yaw);
    sendSpinGoal(return_yaw);
    phase_ = 2;
    return BT::NodeStatus::RUNNING;
  }

  case 2:
  {
    // Add total distance covered in return
    double return_yaw = shortestReturn(max_yaw_);
    cumulative_rotation_ += std::fabs(return_yaw);

    // If full circle already done → skip CW phases
    if (cumulative_rotation_ >= 2 * M_PI - 0.01)
    {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Already rotated full 360° (%.2f rad) → skipping CW phases.",
                  name().c_str(), cumulative_rotation_);
      return BT::NodeStatus::SUCCESS;
    }

    // ---- Phase 3: Spin CW to min_yaw ----
    if (min_yaw_ != 0.0)
    {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Phase 3: spinning CW to %.2f rad", name().c_str(), min_yaw_);
      sendSpinGoal(min_yaw_);
      phase_ = 3;
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Skipping CW phase (min_yaw = 0). Sequence done.",
                  name().c_str());
      return BT::NodeStatus::SUCCESS;
    }
  }

  case 3:
  {
    // ---- Phase 4: Return to 0 from CW ----
    double return_yaw = shortestReturn(min_yaw_);
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Phase 4: returning to 0 rad (shortest path %.2f rad)",
                name().c_str(), return_yaw);
    sendSpinGoal(return_yaw);
    phase_ = 4;
    return BT::NodeStatus::RUNNING;
  }

  case 4:
    // ---- Done ----
    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Spin sequence complete (CCW → 0 → CW → 0).",
                name().c_str());
    return BT::NodeStatus::SUCCESS;

  default:
    return BT::NodeStatus::FAILURE;
  }
}

void Spin::onHalted()
{
  if (client_ptr_ && goal_handle_future_.valid())
  {
    auto handle = goal_handle_future_.get();
    if (handle)
    {
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "[%s] Cancelling spin...", name().c_str());
      client_ptr_->async_cancel_goal(handle);
    }
  }
}

void Spin::sendSpinGoal(double yaw)
{
  NavSpin::Goal goal;
  goal.target_yaw = yaw;
  goal.time_allowance = rclcpp::Duration::from_seconds(spin_duration_);

  auto options = rclcpp_action::Client<NavSpin>::SendGoalOptions();
  options.result_callback = [this](const GoalHandleSpin::WrappedResult &result)
  {
    done_flag_ = true;
    nav_result_ = result.code;
  };

  goal_handle_future_ = client_ptr_->async_send_goal(goal, options);
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