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

// ============================ KeepRunningUntilObjectFound ============================ //

ForEachEvaluationPrompt::ForEachEvaluationPrompt(const std::string &name,
                                                 const BT::NodeConfig &config,
                                                 const rclcpp::Node::SharedPtr &nodePtr)
    : BT::DecoratorNode(name, config), node(nodePtr)
{
    rclcpp::QoS qos(1);
    qos.transient_local();

    subscriber = node->create_subscription<EvaluationEvent>(
        "/evaluation/event", qos,
        std::bind(&ForEachEvaluationPrompt::callbackEvent, this, std::placeholders::_1));

    statusPublisher = node->create_publisher<std_msgs::msg::String>(
        "/evaluation/iteration_status", 10);
}

BT::PortsList ForEachEvaluationPrompt::providedPorts()
{
    return {BT::OutputPort<std::string>("targetObject")};
}

BT::NodeStatus ForEachEvaluationPrompt::tick()
{
    if (!subscribed) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "[ForEachEvaluationPrompt] Waiting for /evaluation/event...");
        return BT::NodeStatus::RUNNING;
    }

    if (currentIndex >= promptTexts.size()) {
        RCLCPP_INFO(node->get_logger(), "[ForEachEvaluationPrompt] All prompts processed.");
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

    RCLCPP_INFO(node->get_logger(), "=== Evaluation Configuration Received ===");
    RCLCPP_INFO(node->get_logger(), "Experiment: %s", msg->experiment_id.c_str());
    RCLCPP_INFO(node->get_logger(), "Episode: %s", msg->episode_id.c_str());
    RCLCPP_INFO(node->get_logger(), "Phase: %s", msg->phase.c_str());
    RCLCPP_INFO(node->get_logger(), "Prompts in this run:");

    for (const auto &prompt : msg->prompt_list.prompt_list) {
        RCLCPP_INFO(node->get_logger(), "  - %s", prompt.text_query.c_str());
        promptTexts.push_back(prompt.text_query);
    }

    RCLCPP_INFO(node->get_logger(), "==========================================");
}
