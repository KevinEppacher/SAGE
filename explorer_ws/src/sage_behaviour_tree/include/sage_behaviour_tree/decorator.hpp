#pragma once

#include <behaviortree_cpp/decorator_node.h>

class KeepRunningUntilObjectFound : public BT::DecoratorNode
{
public:
  KeepRunningUntilObjectFound(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool objectFound = false;
};