#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <sensor_msgs/msg/image.hpp>
#include <multimodal_query_msgs/msg/semantic_prompt.hpp>

class PublishSemanticPrompt : public BT::SyncActionNode
{
public:
    PublishSemanticPrompt(const std::string& name,
                          const BT::NodeConfiguration& config,
                          rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<multimodal_query_msgs::msg::SemanticPrompt>::SharedPtr pub;
    bool published_{false};
    std::string topic_;
};