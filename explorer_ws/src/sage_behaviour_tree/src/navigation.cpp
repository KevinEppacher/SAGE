#include "sage_behaviour_tree/navigation.hpp"
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

// -------------------- Spin -------------------- //

Spin::Spin(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      nodePtr_(std::move(nodePtr)) {}

BT::PortsList Spin::providedPorts()
{
  return {
      BT::InputPort<double>("turnLeftAngle", 0.0, "CCW rotation angle in radians"),
      BT::InputPort<double>("turnRightAngle", 0.0, "CW rotation angle in radians (negative)"),
      BT::InputPort<double>("spinDuration", 15.0, "Spin duration seconds")};
}

double Spin::shortestReturn(double angle)
{
  // Return shortest rotation back to 0 radians
  if (std::fabs(angle) <= M_PI)
    return -angle;

  if (angle > 0.0)
    return 2 * M_PI - angle;
  else
    return -2 * M_PI - angle;
}

BT::NodeStatus Spin::onStart()
{
  getInput("turnLeftAngle", turnLeftAngle_);
  getInput("turnRightAngle", turnRightAngle_);
  getInput("spinDuration", spinDuration_);

  clientPtr_ = rclcpp_action::create_client<NavSpin>(nodePtr_, "/spin");
  if (!clientPtr_->wait_for_action_server(1s))
  {
    RCLCPP_ERROR(nodePtr_->get_logger(),
                 "[%s] Spin action server not available", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

    {
        auto abort_nav_goal = [&](const std::string &action_name)
        {
            try
            {
            auto generic_client =
                rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                    nodePtr_, action_name);

            if (generic_client->wait_for_action_server(100ms))
            {
                RCLCPP_INFO(nodePtr_->get_logger(),
                            "[%s] Aborting existing goal on %s",
                            name().c_str(), action_name.c_str());
                generic_client->async_cancel_all_goals();
            }
            }
            catch (const std::exception &e)
            {
            RCLCPP_WARN(nodePtr_->get_logger(),
                        "[%s] Exception while aborting %s: %s",
                        name().c_str(), action_name.c_str(), e.what());
            }
        };

    abort_nav_goal("/navigate_to_pose");
    abort_nav_goal("/follow_path");
    abort_nav_goal("/compute_path_to_pose");
    }

  doneFlag_ = false;
  phase_ = 0;
  cumulativeRotation_ = 0.0;

  // ---- Skip if both zero ----
  if (turnLeftAngle_ == 0.0 && turnRightAngle_ == 0.0)
  {
    RCLCPP_WARN(nodePtr_->get_logger(),
                "[%s] Both angles are 0 → nothing to spin.", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

    // ---- Check for full-turn optimization ----
    bool fullLeft = std::fabs(turnLeftAngle_) > M_PI;
    bool fullRight = std::fabs(turnRightAngle_) > M_PI;
    double fullTurn = 2 * M_PI;

    if (fullLeft && !fullRight)
    {
        RCLCPP_INFO(nodePtr_->get_logger(),
                    "[%s] Performing full CCW turn (%.2f rad)", name().c_str(),
                    turnLeftAngle_);
        sendSpinGoal(fullTurn);
        phase_ = 99;  // full-turn mode
        return BT::NodeStatus::RUNNING;
    }

    if (fullRight && !fullLeft)
    {
        RCLCPP_INFO(nodePtr_->get_logger(),
                    "[%s] Performing full CW turn (%.2f rad)", name().c_str(),
                    turnRightAngle_);
        sendSpinGoal(-fullTurn);
        phase_ = 99;
        return BT::NodeStatus::RUNNING;
    }

    if (fullLeft && fullRight)
    {
        RCLCPP_INFO(nodePtr_->get_logger(),
                    "[%s] Both angles exceed 180° → defaulting to left turn.", name().c_str());
        sendSpinGoal(fullTurn);
        phase_ = 99;
        return BT::NodeStatus::RUNNING;
    }

  // ---- Normal multi-phase sequence ----
  if (turnLeftAngle_ != 0.0)
  {
    RCLCPP_INFO(nodePtr_->get_logger(),
                "[%s] Phase 1: spinning CCW to %.2f rad", name().c_str(), turnLeftAngle_);
    sendSpinGoal(turnLeftAngle_);
    phase_ = 1;
  }
  else
  {
    RCLCPP_INFO(nodePtr_->get_logger(),
                "[%s] Skipping CCW phase (turnLeftAngle = 0).", name().c_str());
    if (turnRightAngle_ != 0.0)
    {
      RCLCPP_INFO(nodePtr_->get_logger(),
                  "[%s] Phase 3: spinning CW to %.2f rad", name().c_str(), turnRightAngle_);
      sendSpinGoal(turnRightAngle_);
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
  // Wait until current action completes
  if (!doneFlag_)
    return BT::NodeStatus::RUNNING;

  // ---- Full-turn case ----
  if (phase_ == 99)
  {
    if (navResult_ == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(nodePtr_->get_logger(),
                  "[%s] Full 360° spin completed.", name().c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(nodePtr_->get_logger(),
                  "[%s] Full 360° spin failed or canceled.", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Reset flag for normal sequences
  doneFlag_ = false;

  if (navResult_ != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_WARN(nodePtr_->get_logger(),
                "[%s] Spin failed or canceled.", name().c_str());
    return BT::NodeStatus::FAILURE;
  }


  switch (phase_)
  {
  case 1:
  {
    // Phase 2: Return to 0 from CCW
    double returnYaw = shortestReturn(turnLeftAngle_);
    cumulativeRotation_ += std::fabs(turnLeftAngle_);
    RCLCPP_INFO(nodePtr_->get_logger(),
                "[%s] Phase 2: returning to 0 rad (shortest path %.2f rad)",
                name().c_str(), returnYaw);
    sendSpinGoal(returnYaw);
    phase_ = 2;
    return BT::NodeStatus::RUNNING;
  }

  case 2:
  {
    cumulativeRotation_ += std::fabs(shortestReturn(turnLeftAngle_));
    if (cumulativeRotation_ >= 2 * M_PI - 0.01)
    {
      RCLCPP_INFO(nodePtr_->get_logger(),
                  "[%s] Already rotated full 360° (%.2f rad) → skipping CW phases.",
                  name().c_str(), cumulativeRotation_);
      return BT::NodeStatus::SUCCESS;
    }

    if (turnRightAngle_ != 0.0)
    {
      RCLCPP_INFO(nodePtr_->get_logger(),
                  "[%s] Phase 3: spinning CW to %.2f rad", name().c_str(), turnRightAngle_);
      sendSpinGoal(turnRightAngle_);
      phase_ = 3;
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      return BT::NodeStatus::SUCCESS;
    }
  }

  case 3:
  {
    // Phase 4: Return to 0 from CW
    double returnYaw = shortestReturn(turnRightAngle_);
    RCLCPP_INFO(nodePtr_->get_logger(),
                "[%s] Phase 4: returning to 0 rad (shortest path %.2f rad)",
                name().c_str(), returnYaw);
    sendSpinGoal(returnYaw);
    phase_ = 4;
    return BT::NodeStatus::RUNNING;
  }

  case 4:
    RCLCPP_INFO(nodePtr_->get_logger(),
                "[%s] Spin sequence complete (CCW → 0 → CW → 0).", name().c_str());
    return BT::NodeStatus::SUCCESS;

  default:
    return BT::NodeStatus::FAILURE;
  }
}

void Spin::onHalted()
{
  if (clientPtr_ && goalHandleFuture_.valid())
  {
    auto handle = goalHandleFuture_.get();
    if (handle)
    {
      RCLCPP_INFO(nodePtr_->get_logger(),
                  "[%s] Cancelling spin...", name().c_str());
      clientPtr_->async_cancel_goal(handle);
    }
  }
}

void Spin::sendSpinGoal(double yaw)
{
  NavSpin::Goal goal;
  goal.target_yaw = yaw;
  goal.time_allowance = rclcpp::Duration::from_seconds(spinDuration_);

  auto options = rclcpp_action::Client<NavSpin>::SendGoalOptions();
  options.result_callback = [this](const GoalHandleSpin::WrappedResult &result)
  {
    doneFlag_ = true;
    navResult_ = result.code;
  };

  goalHandleFuture_ = clientPtr_->async_send_goal(goal, options);
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