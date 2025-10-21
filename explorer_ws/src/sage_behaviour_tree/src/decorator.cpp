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