#include "sage_behaviour_tree/decorator.hpp"

KeepRunningUntilObjectFound::KeepRunningUntilObjectFound(
    const std::string& name,
    const BT::NodeConfiguration& config)
    : BT::DecoratorNode(name, config)
{
}

BT::PortsList KeepRunningUntilObjectFound::providedPorts()
{
  return {};
}

BT::NodeStatus KeepRunningUntilObjectFound::tick()
{
  if (objectFound)
  {
    // Stop immediately once object found
    return BT::NodeStatus::SUCCESS;
  }

  const BT::NodeStatus childStatus = child_node_->executeTick();

  switch (childStatus)
  {
    case BT::NodeStatus::SUCCESS:
      objectFound = true;
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE:
      // Failure of the fallback means exploration failed
      return BT::NodeStatus::FAILURE;

    case BT::NodeStatus::RUNNING:
    default:
      return BT::NodeStatus::RUNNING;
  }
}