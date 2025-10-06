#include "sage_behaviour_tree/utils.hpp"

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