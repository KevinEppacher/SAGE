#include "sage_behaviour_tree/decorator.hpp"

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
      BT::InputPort<bool>("any_exploration_nodes", true, "True if frontiers or exploration nodes remain")};
}

BT::NodeStatus KeepRunningUntilObjectFound::tick()
{
  // Perform one-time default initialization
  if (!initialized)
  {
    // Initialize defaults if not on blackboard
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

  // Read the current blackboard values
  getInput("object_found", objectFound);
  getInput("any_exploration_nodes", anyExplorationNodes);

  // Decision logic
  if (objectFound)
  {
    return BT::NodeStatus::SUCCESS;
  }

  if (!anyExplorationNodes)
  {
    return BT::NodeStatus::FAILURE;
  }

  // Otherwise, keep child running
  const BT::NodeStatus child_status = child_node_->executeTick();

  // Keep looping until mission state changes
  if (child_status == BT::NodeStatus::SUCCESS || child_status == BT::NodeStatus::FAILURE)
  {
    return BT::NodeStatus::RUNNING;
  }

  return child_status;
}

// ============================ ForEachEvaluationPrompt ============================ //

ForEachEvaluationPrompt::ForEachEvaluationPrompt(const std::string &name,
                                                 const BT::NodeConfig &config,
                                                 const rclcpp::Node::SharedPtr &nodePtr)
    : BT::DecoratorNode(name, config), node(nodePtr)
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

    if (!subscribed) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "[%s] Waiting for %s...", name().c_str(), evaluation_event_topic_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // ...existing code...
    if (currentIndex >= promptTexts.size()) {
        RCLCPP_INFO(node->get_logger(), "[%s] All prompts processed.", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    std::string currentPrompt = promptTexts[currentIndex];
    setOutput("targetObject", currentPrompt);

    BT::NodeStatus childStatus = child_node_->executeTick();

    if (childStatus == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
    }

    std_msgs::msg::String msg;
    msg.data = "[ForEachEvaluationPrompt] Object '" + currentPrompt +
               "' finished with status: " +
               (childStatus == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    statusPublisher->publish(msg);

    currentIndex++;
    
    return BT::NodeStatus::RUNNING;
}

void ForEachEvaluationPrompt::callbackEvent(const EvaluationEvent::SharedPtr msg)
{
    if (subscribed) return;
    subscribed = true;

    RCLCPP_INFO(node->get_logger(), "[%s] === Evaluation Configuration Received ===", name().c_str());
    RCLCPP_INFO(node->get_logger(), "[%s] Experiment: %s", name().c_str(), msg->experiment_id.c_str());
    RCLCPP_INFO(node->get_logger(), "[%s] Episode: %s", name().c_str(), msg->episode_id.c_str());
    RCLCPP_INFO(node->get_logger(), "[%s] Phase: %s", name().c_str(), msg->phase.c_str());
    RCLCPP_INFO(node->get_logger(), "[%s] Prompts in this run:", name().c_str());

    for (const auto &prompt : msg->prompt_list.prompt_list) {
        RCLCPP_INFO(node->get_logger(), "[%s]   - %s", name().c_str(), prompt.text_query.c_str());
        promptTexts.push_back(prompt.text_query);
    }

    RCLCPP_INFO(node->get_logger(), "[%s] ==========================================", name().c_str());
}

void ForEachEvaluationPrompt::initCommsFromPorts()
{
    if (commReady) {
        return;
    }

    (void)getInput<std::string>("evaluation_event_topic", evaluation_event_topic_);
    (void)getInput<std::string>("iteration_status_topic", iteration_status_topic_);

    rclcpp::QoS qos(1);
    qos.transient_local();
    subscriber = node->create_subscription<EvaluationEvent>(
        evaluation_event_topic_, qos,
        std::bind(&ForEachEvaluationPrompt::callbackEvent, this, std::placeholders::_1));

    statusPublisher = node->create_publisher<std_msgs::msg::String>(
        iteration_status_topic_, 10);

    RCLCPP_INFO(node->get_logger(), "[%s] Using topics: event='%s', status='%s'",
                name().c_str(), evaluation_event_topic_.c_str(), iteration_status_topic_.c_str());
    commReady = true;
}