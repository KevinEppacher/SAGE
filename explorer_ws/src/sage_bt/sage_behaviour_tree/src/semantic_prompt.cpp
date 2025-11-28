#include "sage_behaviour_tree/semantic_prompt.hpp"
#include "sage_behaviour_tree/colors.hpp"

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
    // Read the topic port
    std::string port_topic;
    if (getInput<std::string>("topic", port_topic) && !port_topic.empty()) {
        topic_ = port_topic;
    }

    // Lazily create publisher
    if (!pub) 
    {
        rclcpp::QoS qosSemanticPrompt(rclcpp::KeepLast(1));
        qosSemanticPrompt.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qosSemanticPrompt.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        pub = node->create_publisher<multimodal_query_msgs::msg::SemanticPrompt>(topic_, qosSemanticPrompt);
        RCLCPP_INFO(node->get_logger(),
                    ORANGE "[%s] Publisher created on topic '%s'" RESET,
                    name().c_str(), topic_.c_str());
    }

    // Get the current query
    std::string text;
    if (!getInput<std::string>("text_query", text)) {
        RCLCPP_ERROR(node->get_logger(),
                     RED "[%s] Missing required port 'text_query'" RESET,
                     name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Optional: only republish if query changed
    static std::string last_query;
    if (text == last_query) {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[%s] Query unchanged — skipping publish." RESET,
                    name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // Build and publish message
    multimodal_query_msgs::msg::SemanticPrompt msg;
    msg.header.stamp = node->now();
    msg.text_query = text;

    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(),
                GREEN "[%s] Published SemanticPrompt: '%s'" RESET,
                name().c_str(), text.c_str());

    last_query = text;
    return BT::NodeStatus::SUCCESS;
}
