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
        BT::InputPort<int>(
            "max_missed_ticks", 10, "Max ticks to wait for a message"),
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "graph_nodes", "Best detected GraphNode")
    };
}

BT::NodeStatus IsDetected::tick()
{
    if (!received_message_) return BT::NodeStatus::RUNNING;

    double threshold = 0.8;
    getInput("detection_threshold", threshold);

    double max_score = -1.0;
    std::shared_ptr<graph_node_msgs::msg::GraphNode> best_node = nullptr;
    for (auto &n : latest_msg_->nodes) {
        if (n.score > max_score) {
            max_score = n.score;
            best_node = std::make_shared<graph_node_msgs::msg::GraphNode>(n);
        }
    }

    if (!best_node) return BT::NodeStatus::FAILURE;
    setOutput("graph_nodes", best_node);

    return (max_score >= threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
