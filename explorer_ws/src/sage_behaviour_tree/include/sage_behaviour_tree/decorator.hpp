#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include "multimodal_query_msgs/msg/semantic_prompt_array.hpp"
#include "evaluator_msgs/msg/evaluation_event.hpp"

// ============================ KeepRunningUntilObjectFound ============================ //

class KeepRunningUntilObjectFound : public BT::DecoratorNode
{
    public:
        KeepRunningUntilObjectFound(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        // Called on every tick
        BT::NodeStatus tick() override;

        private:
        bool objectFound = false;
        bool anyExplorationNodes = true;
        bool initialized = false;  // for first-tick initialization
};

// ============================ KeepRunningUntilObjectFound ============================ //

using evaluator_msgs::msg::EvaluationEvent;

class ForEachEvaluationPrompt : public BT::DecoratorNode
{
public:
    ForEachEvaluationPrompt(const std::string &name,
                            const BT::NodeConfig &config,
                            const rclcpp::Node::SharedPtr &node);
    ~ForEachEvaluationPrompt() override = default;

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void callbackEvent(const evaluator_msgs::msg::EvaluationEvent::SharedPtr msg);

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<evaluator_msgs::msg::EvaluationEvent>::SharedPtr subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher;

    bool subscribed{false};
    std::vector<std::string> promptTexts;
    size_t currentIndex{0};
};