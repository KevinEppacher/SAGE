#include "sage_behaviour_tree/detection.hpp"

IsDetected::IsDetected(const std::string& name,
                       const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr node_ptr)
    : BT::ConditionNode(name, config), node_ptr_(std::move(node_ptr))
{
    std::string topic;
    getInput<std::string>("detection_graph_node_topic", topic);
    sub_ = node_ptr_->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        topic, 10,
        [this](const graph_node_msgs::msg::GraphNodeArray::SharedPtr msg) {
            latest_msg_ = msg;
            received_message_ = true;
            missed_ticks_ = 0;
        });
}

BT::PortsList IsDetected::providedPorts()
{
    return {
        BT::InputPort<double>(
            "detection_threshold", 0.8, "Detection threshold"),
        BT::InputPort<std::string>(
            "detection_graph_node_topic",
            "/detection_graph_nodes/graph_nodes",
            "GraphNodeArray topic"),
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "graph_nodes", "Best detected GraphNode")
    };
}

BT::NodeStatus IsDetected::tick()
{
    if (!received_message_) 
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] No message received yet on the topic %s. Returning RUNNING", name().c_str(),
                    sub_->get_topic_name());
        return BT::NodeStatus::RUNNING;
    }

    double threshold = 0.8;
    getInput("detection_threshold", threshold);

    double max_score = -1.0;
    std::shared_ptr<graph_node_msgs::msg::GraphNode> best_node = nullptr;
    for (auto &n : latest_msg_->nodes) 
    {
        if (n.score > max_score) 
        {
            max_score = n.score;
            best_node = std::make_shared<graph_node_msgs::msg::GraphNode>(n);
        }
    }

    if (!best_node) 
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] No nodes in the latest message. Returning FAILURE", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    setOutput("graph_nodes", best_node);

    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Best node ID: %d with score: %.2f. Returning SUCCESS", name().c_str(), best_node->id, best_node->score);


    if(max_score >= threshold) {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Detection threshold met: %.2f >= %.2f", name().c_str(), max_score, threshold);
    } else {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Detection threshold not met: %.2f < %.2f", name().c_str(), max_score, threshold);
    }

    return (max_score >= threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
