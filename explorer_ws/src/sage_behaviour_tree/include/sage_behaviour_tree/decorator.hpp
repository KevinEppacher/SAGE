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

// ============================ ForEachEvaluationPrompt ============================ //

using evaluator_msgs::msg::EvaluationEvent;

class ForEachEvaluationPrompt : public BT::DecoratorNode
{
public:
    ForEachEvaluationPrompt(const std::string &name,
                            const BT::NodeConfig &config,
                            const rclcpp::Node::SharedPtr &nodePtr);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void callbackEvent(const EvaluationEvent::SharedPtr msg);
    // New: encapsulate lazy comms init
    void initCommsFromPorts();

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<EvaluationEvent>::SharedPtr subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher;

    bool subscribed{false};
    size_t currentIndex{0};
    std::vector<std::string> promptTexts;

    // Comms config/state
    bool commReady{false};
    std::string evaluation_event_topic_{"/evaluation/event"};
    std::string iteration_status_topic_{"/evaluation/iteration_status"};
};