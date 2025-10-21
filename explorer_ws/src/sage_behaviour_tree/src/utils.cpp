#include "sage_behaviour_tree/utils.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

SetParameterNode::SetParameterNode(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name, config),
      node_ptr_(std::move(node_ptr))
{}

BT::PortsList SetParameterNode::providedPorts()
{
    return {
        BT::InputPort<std::string>("target_node", "Node name to set parameter on"),
        BT::InputPort<std::string>("parameter_name", "Name of the parameter to set"),
        BT::InputPort<double>("value", "Value to set for the parameter")
    };
}

BT::NodeStatus SetParameterNode::tick()
{
    std::string target_node, param_name;
    double value;

    if (!getInput("target_node", target_node) ||
        !getInput("parameter_name", param_name) ||
        !getInput("value", value))
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "[%s] Missing required input port(s)", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Create a lightweight helper node for service calls
    auto helper_node = std::make_shared<rclcpp::Node>("param_client_helper");
    auto client = helper_node->create_client<rcl_interfaces::srv::SetParameters>(
        target_node + "/set_parameters");

    if (!client->wait_for_service(2s)) {
        RCLCPP_ERROR(node_ptr_->get_logger(),
                    "[%s] Parameter service not available for %s",
                    name().c_str(), target_node.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(node_ptr_->get_logger(),
                "[%s] Setting %s/%s = %.3f",
                name().c_str(), target_node.c_str(), param_name.c_str(), value);

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(rclcpp::Parameter(param_name, value).to_parameter_msg());

    auto future = client->async_send_request(request);

    // Use a separate executor for this one-shot node
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(helper_node);

    if (exec.spin_until_future_complete(future, 2s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_DEBUG(node_ptr_->get_logger(),
                    "[%s] Successfully updated parameter %s to %.3f",
                    name().c_str(), param_name.c_str(), value);
        exec.remove_node(helper_node);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[%s] Timeout or failure setting parameter %s on %s",
                    name().c_str(), param_name.c_str(), target_node.c_str());
        exec.remove_node(helper_node);
        return BT::NodeStatus::FAILURE;
    }
}



// -------------------- SeekoutGraphNodes -------------------- //

SeekoutGraphNodes::SeekoutGraphNodes(const std::string &name,
                                     const BT::NodeConfiguration &config,
                                     rclcpp::Node::SharedPtr nodePtr)
    : BT::StatefulActionNode(name, config),
      nodePtr_(std::move(nodePtr)),
      robot_(std::make_unique<Robot>(nodePtr_))
{

}

BT::PortsList SeekoutGraphNodes::providedPorts()
{
  return {
      BT::InputPort<std::string>("graph_node_topic", "/fused/exploration_graph_nodes/graph_nodes"),
      BT::InputPort<std::string>("map_frame", "map"),
      BT::InputPort<std::string>("robot_frame", "base_link"),
      BT::InputPort<double>("sight_horizon", std::to_string(10.0), "Horizon distance (m)"),
      BT::InputPort<double>("min_yaw_default", std::to_string(-M_PI/2), "Default min yaw (rad)"),
      BT::InputPort<double>("max_yaw_default", std::to_string(M_PI/2), "Default max yaw (rad)"),
      BT::OutputPort<double>("min_yaw"),
      BT::OutputPort<double>("max_yaw")
  };
}


void SeekoutGraphNodes::initSubscription(const std::string &topic)
{
  if (sub_)
    return;

  sub_ = nodePtr_->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
      topic, 10,
      [this](graph_node_msgs::msg::GraphNodeArray::SharedPtr msg)
      {
        latestMsg_ = std::move(msg);
        receivedMsg_ = true;
      });

  RCLCPP_INFO(nodePtr_->get_logger(), "[%s] Subscribed to %s", name().c_str(), topic.c_str());
}

BT::NodeStatus SeekoutGraphNodes::onStart()
{
  // Read parameters
  getInput("graph_node_topic", topic_);
  getInput("map_frame", mapFrame_);
  getInput("robot_frame", robotFrame_);
  getInput("sight_horizon", horizon_);
  getInput("min_yaw_default", minDef_);
  getInput("max_yaw_default", maxDef_);

  initSubscription(topic_);

  receivedMsg_ = false;
  latestMsg_.reset();
  startTime_ = nodePtr_->now();

  RCLCPP_INFO(nodePtr_->get_logger(), "[%s] Waiting for GraphNodes and robot pose...", name().c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SeekoutGraphNodes::onRunning()
{
    startTime_ = clock_->now();

    // later
    rclcpp::Time now = clock_->now();
    rclcpp::Duration elapsed = now - startTime_;
    double elapsedSec = elapsed.nanoseconds() * 1e-9;
    if (elapsedSec > timeoutSec_) 
    {
        RCLCPP_ERROR(nodePtr_->get_logger(),
                    "[%s] Timeout (%.1f s) waiting for data (limit %.0f s)",
                    name().c_str(), elapsedSec, timeoutSec_);
        return BT::NodeStatus::FAILURE;
    }

  geometry_msgs::msg::Pose robotPose;
  if (!robot_->getPose(robotPose, mapFrame_, robotFrame_))
  {
    RCLCPP_DEBUG(nodePtr_->get_logger(), "[%s] Robot pose unavailable yet.", name().c_str());
    return BT::NodeStatus::RUNNING;
  }

  if (!receivedMsg_ || !latestMsg_)
  {
    RCLCPP_DEBUG(nodePtr_->get_logger(), "[%s] No GraphNodes yet.", name().c_str());
    return BT::NodeStatus::RUNNING;
  }

  // Compute yaw range
  double minYaw{minDef_}, maxYaw{maxDef_};
  if (!computeYawRange(robotPose, horizon_, minYaw, maxYaw))
  {
    RCLCPP_WARN(nodePtr_->get_logger(),
                "[%s] No nodes within horizon → using defaults.", name().c_str());
    setOutput("min_yaw", minDef_);
    setOutput("max_yaw", maxDef_);
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("min_yaw", minYaw);
  setOutput("max_yaw", maxYaw);

  RCLCPP_INFO(nodePtr_->get_logger(),
              "[%s] Computed visible yaw span: [%.1f°, %.1f°]",
              name().c_str(), minYaw * 180.0 / M_PI, maxYaw * 180.0 / M_PI);

  return BT::NodeStatus::SUCCESS;
}

void SeekoutGraphNodes::onHalted()
{
  RCLCPP_INFO(nodePtr_->get_logger(), "[%s] Halted.", name().c_str());
}

bool SeekoutGraphNodes::computeYawRange(
    const geometry_msgs::msg::Pose &robotPose,
    double sightHorizon,
    double &minYaw,
    double &maxYaw)
{
  if (!latestMsg_)
    return false;

  const double rx = robotPose.position.x;
  const double ry = robotPose.position.y;

  // --- Extract robot yaw from quaternion ---
  tf2::Quaternion q;
  tf2::fromMsg(robotPose.orientation, q);
  double roll, pitch, robotYaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, robotYaw);

  std::vector<double> yaws;
  yaws.reserve(latestMsg_->nodes.size());

  for (const auto &n : latestMsg_->nodes)
  {
    // Transform node position to robot-centered coordinates
    double dx = n.position.x - rx;
    double dy = n.position.y - ry;
    double dist = std::hypot(dx, dy);
    if (dist > sightHorizon)
      continue;

    // Global yaw of node in map frame
    double globalYaw = std::atan2(dy, dx);

    // Convert to local robot frame (robot heading = 0 rad)
    double relativeYaw = globalYaw - robotYaw;

    // Normalize to [-pi, pi]
    if (relativeYaw > M_PI)
      relativeYaw -= 2 * M_PI;
    else if (relativeYaw < -M_PI)
      relativeYaw += 2 * M_PI;

    yaws.push_back(relativeYaw);
  }

  if (yaws.empty())
    return false;

  // --- Determine visible angular span in robot frame ---
  auto [minIt, maxIt] = std::minmax_element(yaws.begin(), yaws.end());
  minYaw = *minIt;
  maxYaw = *maxIt;

  RCLCPP_DEBUG(nodePtr_->get_logger(),
               "[%s] Local yaw range (robot frame): min=%.2f°, max=%.2f°",
               name().c_str(), minYaw * 180.0 / M_PI, maxYaw * 180.0 / M_PI);

  return true;
}