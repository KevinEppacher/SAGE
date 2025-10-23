#include "sage_behaviour_tree/semantic_prompt.hpp"

PublishSemanticPrompt::PublishSemanticPrompt(const std::string& name,
                                             const BT::NodeConfiguration& config,
                                             rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node(node),
  published_(false),
  topic_("/user_prompt")
{

}

BT::PortsList PublishSemanticPrompt::providedPorts()
{
    return {
        BT::InputPort<std::string>("text_query", "Text query to publish as SemanticPrompt"),
        BT::InputPort<std::string>("topic", "/user_prompt", "Topic to publish the SemanticPrompt message on")
    };
}

BT::NodeStatus PublishSemanticPrompt::tick()
{
    if (published_) {
        return BT::NodeStatus::SUCCESS;
    }

    // Read (optional) topic input port and set up publisher if not already created
    std::string port_topic;
    if (getInput<std::string>("topic", port_topic)) {
        if (!port_topic.empty()) {
            topic_ = port_topic;
        }
    }

    if (!pub) {
        pub = node->create_publisher<multimodal_query_msgs::msg::SemanticPrompt>(topic_, 10);
        RCLCPP_INFO(node->get_logger(), "PublishSemanticPrompt: publisher created on topic '%s'", topic_.c_str());
    }

    std::string text;
    if (!getInput<std::string>("text_query", text)) {
        RCLCPP_ERROR(node->get_logger(), "PublishSemanticPrompt: missing required port 'text_query'");
        return BT::NodeStatus::FAILURE;
    }

    multimodal_query_msgs::msg::SemanticPrompt msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "";
    msg.text_query = text;
    // image_query left empty for now

    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published SemanticPrompt to '%s': '%s'", topic_.c_str(), text.c_str());

    published_ = true;
    return BT::NodeStatus::SUCCESS;
}