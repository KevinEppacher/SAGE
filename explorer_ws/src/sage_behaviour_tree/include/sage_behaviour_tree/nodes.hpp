#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <thread>

// Simple synchronous action that prints a message.
class Say : public BT::SyncActionNode {
public:
  Say(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("text") };
  }
  BT::NodeStatus tick() override {
    auto msg = getInput<std::string>("text").value_or("no text");
    std::cout << "[Say] " << msg << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// Simple synchronous "wait" node.
class WaitMs : public BT::SyncActionNode {
public:
  WaitMs(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<int>("ms", 100) };
  }
  BT::NodeStatus tick() override {
    int ms = getInput<int>("ms").value_or(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    return BT::NodeStatus::SUCCESS;
  }
};
