#pragma once

#include <behaviortree_cpp/decorator_node.h>

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