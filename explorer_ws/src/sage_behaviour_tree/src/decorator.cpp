#include "sage_behaviour_tree/decorator.hpp"
#include "sage_behaviour_tree/colors.hpp"

#ifndef ORANGE
#define ORANGE "\033[38;5;208m"
#endif

// ============================ KeepRunningUntilObjectFound ============================ //

KeepRunningUntilObjectFound::KeepRunningUntilObjectFound(
    const std::string& name,
    const BT::NodeConfiguration& config)
    : BT::DecoratorNode(name, config)
{
}

BT::PortsList KeepRunningUntilObjectFound::providedPorts()
{
    return {
        BT::InputPort<bool>("object_found", false, "True if target object was detected"),
        BT::InputPort<bool>("any_exploration_nodes", true, "True if frontiers or exploration nodes remain")
    };
}

BT::NodeStatus KeepRunningUntilObjectFound::tick()
{
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    if (!initialized)
    {
        if (!getInput("object_found", objectFound))
        {
            objectFound = false;
            setOutput("object_found", objectFound);
        }

        if (!getInput("any_exploration_nodes", anyExplorationNodes))
        {
            anyExplorationNodes = true;
            setOutput("any_exploration_nodes", anyExplorationNodes);
        }

        initialized = true;
    }

    getInput("object_found", objectFound);
    getInput("any_exploration_nodes", anyExplorationNodes);

    if (objectFound)
    {
        RCLCPP_INFO(rclcpp::get_logger("KeepRunningUntilObjectFound"),
                    GREEN "[%s] Object found → SUCCESS" RESET, name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    if (!anyExplorationNodes)
    {
        RCLCPP_WARN(rclcpp::get_logger("KeepRunningUntilObjectFound"),
                    RED "[%s] No exploration nodes remaining → FAILURE" RESET, name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    const BT::NodeStatus childStatus = child_node_->executeTick();

    if (childStatus == BT::NodeStatus::SUCCESS || childStatus == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("KeepRunningUntilObjectFound"),
                             steady_clock, 2000,
                             ORANGE "[%s] Child finished, continuing loop (RUNNING)" RESET,
                             name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("KeepRunningUntilObjectFound"),
    //                      steady_clock, 2000,
    //                      ORANGE "[%s] Running child tick..." RESET,
    //                      name().c_str());
    return childStatus;
}


// ============================ ForEachEvaluationPrompt ============================ //

ForEachEvaluationPrompt::ForEachEvaluationPrompt(const std::string &name,
                                                 const BT::NodeConfig &config,
                                                 const rclcpp::Node::SharedPtr &nodePtr)
    : BT::DecoratorNode(name, config),
      node(nodePtr)
{
}

BT::PortsList ForEachEvaluationPrompt::providedPorts()
{
    return {
        BT::OutputPort<std::string>("targetObject"),
        BT::InputPort<std::string>("evaluation_event_topic", "/evaluation/event",
                                   "Topic to subscribe for EvaluationEvent"),
        BT::InputPort<std::string>("iteration_status_topic", "/evaluation/iteration_status",
                                   "Topic to publish iteration status")
    };
}

BT::NodeStatus ForEachEvaluationPrompt::tick()
{
    initCommsFromPorts();

    if (!subscribed)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             ORANGE "[%s] Waiting for %s..." RESET,
                             name().c_str(), evaluation_event_topic_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    if (currentIndex >= promptTexts.size())
    {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[%s] All prompts processed → SUCCESS" RESET,
                    name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    std::string currentPrompt = promptTexts[currentIndex];
    setOutput("targetObject", currentPrompt);

    BT::NodeStatus childStatus = child_node_->executeTick();

    if (childStatus == BT::NodeStatus::RUNNING)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
                             ORANGE "[%s] Processing prompt '%s'..." RESET,
                             name().c_str(), currentPrompt.c_str());
        return BT::NodeStatus::RUNNING;
    }

    std_msgs::msg::String msg;
    msg.data = "[ForEachEvaluationPrompt] Object '" + currentPrompt +
               "' finished with status: " +
               (childStatus == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    statusPublisher->publish(msg);

    RCLCPP_INFO(node->get_logger(),
                (childStatus == BT::NodeStatus::SUCCESS
                     ? GREEN "[%s] Prompt '%s' → SUCCESS" RESET
                     : RED "[%s] Prompt '%s' → FAILURE" RESET),
                name().c_str(), currentPrompt.c_str());

    currentIndex++;
    return BT::NodeStatus::RUNNING;
}

void ForEachEvaluationPrompt::callbackEvent(const EvaluationEvent::SharedPtr msg)
{
    if (subscribed)
        return;
    subscribed = true;

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] === Evaluation Configuration Received ===" RESET,
                name().c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Experiment: %s" RESET,
                name().c_str(), msg->experiment_id.c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Episode: %s" RESET,
                name().c_str(), msg->episode_id.c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Phase: %s" RESET,
                name().c_str(), msg->phase.c_str());
    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Prompts in this run:" RESET,
                name().c_str());

    for (const auto &prompt : msg->prompt_list.prompt_list)
    {
        RCLCPP_INFO(node->get_logger(),
                    ORANGE "[%s]   - %s" RESET,
                    name().c_str(), prompt.text_query.c_str());
        promptTexts.push_back(prompt.text_query);
    }

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] ==========================================" RESET,
                name().c_str());
}

void ForEachEvaluationPrompt::initCommsFromPorts()
{
    if (commReady)
        return;

    (void)getInput<std::string>("evaluation_event_topic", evaluation_event_topic_);
    (void)getInput<std::string>("iteration_status_topic", iteration_status_topic_);

    rclcpp::QoS qos(1);
    qos.transient_local();
    subscriber = node->create_subscription<EvaluationEvent>(
        evaluation_event_topic_, qos,
        std::bind(&ForEachEvaluationPrompt::callbackEvent, this, std::placeholders::_1));

    statusPublisher = node->create_publisher<std_msgs::msg::String>(
        iteration_status_topic_, 10);

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Using topics: event='%s', status='%s'" RESET,
                name().c_str(), evaluation_event_topic_.c_str(), iteration_status_topic_.c_str());

    commReady = true;
}
