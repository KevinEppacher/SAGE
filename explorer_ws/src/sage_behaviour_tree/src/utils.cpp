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

SeekoutGraphNodes::SeekoutGraphNodes(const std::string& name,
                                     const BT::NodeConfiguration& config,
                                     rclcpp::Node::SharedPtr node_ptr)
: BT::SyncActionNode(name, config),
  node_ptr_(std::move(node_ptr)),
  robot_(std::make_unique<Robot>(node_ptr_))
{
    marker_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>(
        "seekout_debug_marker", 10);
}

BT::PortsList SeekoutGraphNodes::providedPorts()
{
    return {
        BT::InputPort<std::string>("graph_node_topic",
            "/fused/exploration_graph_nodes/graph_nodes", "GraphNodeArray topic"),
        BT::InputPort<std::string>("map_frame", "map", "Map frame"),
        BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame"),
        BT::InputPort<double>("sight_horizon", 10.0, "Horizon distance (m)"),
        BT::InputPort<double>("min_yaw_default", -M_PI, "Default min yaw (rad)"),
        BT::InputPort<double>("max_yaw_default",  M_PI, "Default max yaw (rad)"),
        BT::OutputPort<double>("min_yaw", "Computed min yaw (rad)"),
        BT::OutputPort<double>("max_yaw", "Computed max yaw (rad)")
    };
}

void SeekoutGraphNodes::initSubscription(const std::string& topic)
{
    if (sub_) return;

    sub_ = node_ptr_->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        topic, rclcpp::QoS(10),
        [this](graph_node_msgs::msg::GraphNodeArray::SharedPtr msg)
        {
            latest_msg_ = std::move(msg);
            received_message_ = true;
            missed_ticks_ = 0;
        });

    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Subscribed: %s", name().c_str(), topic.c_str());
}

BT::NodeStatus SeekoutGraphNodes::tick()
{
    std::string topic, map_frame, robot_frame;
    double horizon{10.0}, min_def{-M_PI}, max_def{M_PI};

    getInput("graph_node_topic", topic);
    getInput("map_frame", map_frame);
    getInput("robot_frame", robot_frame);
    getInput("sight_horizon", horizon);
    getInput("min_yaw_default", min_def);
    getInput("max_yaw_default", max_def);

    // Lazy subscription
    initSubscription(topic);

    // Get robot pose
    geometry_msgs::msg::Pose robot_pose;
    if (!robot_->getPose(robot_pose, map_frame, robot_frame))
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Pose unavailable → using defaults.", name().c_str());
        setOutput("min_yaw", min_def);
        setOutput("max_yaw", max_def);
        publishMarker(robot_pose, horizon, min_def, max_def);
        return BT::NodeStatus::SUCCESS;
    }

    // Timeout / missed messages
    if (!received_message_ || !latest_msg_)
    {
        if (++missed_ticks_ >= MAX_MISSED_TICKS)
        {
            RCLCPP_WARN(node_ptr_->get_logger(), "[%s] No new GraphNodeArray for %d ticks → defaults.",
                        name().c_str(), missed_ticks_);
            setOutput("min_yaw", min_def);
            setOutput("max_yaw", max_def);
            publishMarker(robot_pose, horizon, min_def, max_def);
        }
        return BT::NodeStatus::SUCCESS;
    }

    // Compute yaw span
    double min_yaw{min_def}, max_yaw{max_def};
    if (!computeYawRange(robot_pose, horizon, min_yaw, max_yaw))
    {
        setOutput("min_yaw", min_def);
        setOutput("max_yaw", max_def);
        publishMarker(robot_pose, horizon, min_def, max_def);
        return BT::NodeStatus::SUCCESS;
    }

    // Publish marker visualization
    publishMarker(robot_pose, horizon, min_yaw, max_yaw);

    setOutput("min_yaw", min_yaw);
    setOutput("max_yaw", max_yaw);

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Visible span: [%.1f°, %.1f°]",
                name().c_str(), min_yaw * 180.0 / M_PI, max_yaw * 180.0 / M_PI);

    return BT::NodeStatus::SUCCESS;
}

bool SeekoutGraphNodes::computeYawRange(const geometry_msgs::msg::Pose& robot_pose,
                                        double sight_horizon,
                                        double& min_yaw,
                                        double& max_yaw)
{
    const double rx = robot_pose.position.x;
    const double ry = robot_pose.position.y;

    std::vector<double> yaws;
    for (const auto& n : latest_msg_->nodes)
    {
        const double dx = n.position.x - rx;
        const double dy = n.position.y - ry;
        const double dist = std::hypot(dx, dy);
        if (dist <= sight_horizon)
            yaws.push_back(std::atan2(dy, dx));
    }

    if (yaws.empty()) return false;

    auto [min_it, max_it] = std::minmax_element(yaws.begin(), yaws.end());
    min_yaw = *min_it;
    max_yaw = *max_it;
    return true;
}

void SeekoutGraphNodes::publishMarker(const geometry_msgs::msg::Pose& robot_pose,
                                      double sight_horizon,
                                      double min_yaw,
                                      double max_yaw)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_ptr_->now();
    marker.ns = "seekout_debug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.03;
    marker.color.r = 0.0;
    marker.color.g = 0.9;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Draw horizon circle
    const int N = 64;
    for (int i = 0; i <= N; ++i)
    {
        double theta = i * 2 * M_PI / N;
        geometry_msgs::msg::Point p;
        p.x = robot_pose.position.x + sight_horizon * std::cos(theta);
        p.y = robot_pose.position.y + sight_horizon * std::sin(theta);
        p.z = 0.05;
        marker.points.push_back(p);
    }

    // Add direction line
    double mid_yaw = 0.5 * (min_yaw + max_yaw);
    geometry_msgs::msg::Point center, end;
    center.x = robot_pose.position.x;
    center.y = robot_pose.position.y;
    center.z = 0.05;
    end.x = robot_pose.position.x + sight_horizon * std::cos(mid_yaw);
    end.y = robot_pose.position.y + sight_horizon * std::sin(mid_yaw);
    end.z = 0.05;

    marker.points.push_back(center);
    marker.points.push_back(end);

    marker_pub_->publish(marker);
}