#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>

// Minimal leaf nodes
class Init : public BT::SyncActionNode {
public:
  Init(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { std::cout << "[Init] ok\n"; return BT::NodeStatus::SUCCESS; }
};
class Work : public BT::SyncActionNode {
public:
  Work(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { std::cout << "[Work] done\n"; return BT::NodeStatus::SUCCESS; }
};
class WaitMs : public BT::SyncActionNode {
public:
  WaitMs(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<int>("ms") }; }
  BT::NodeStatus tick() override {
    int ms = getInput<int>("ms").value_or(300);
    std::cout << "[WaitMs] sleeping for " << ms << " ms\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    return BT::NodeStatus::SUCCESS;
  }
};

class SageBehaviorTreeNode : public rclcpp::Node {
public:
  SageBehaviorTreeNode() : rclcpp::Node("sage_behavior_tree_node") 
  {
    // Declare parameters (only once each)
    this->declare_parameter<std::string>("tree_xml_file", "");
    this->declare_parameter<std::string>("location_file", "");

    std::string xml = this->get_parameter("tree_xml_file").as_string();
    if (xml.empty()) {
      const auto share = ament_index_cpp::get_package_share_directory("sage_behaviour_tree");
      xml = share + std::string("/bt_xml/demo.xml");
    }

    const std::string location_file = this->get_parameter("location_file").as_string();
    RCLCPP_INFO(this->get_logger(), "Using tree XML file: %s", xml.c_str());

    // Build tree
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Init>("Init");
    factory.registerNodeType<Work>("Work");
    factory.registerNodeType<WaitMs>("WaitMs");

    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::string>("location_file", location_file);

    tree_ = factory.createTreeFromFile(xml, blackboard);

    // Print static structure once
    BT::printTreeRecursively(tree_.rootNode());

    // One Groot2 publisher
    publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);

    // Tick at fixed rate
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        BT::NodeStatus tree_status = tree_.tickOnce();
        if (tree_status != BT::NodeStatus::RUNNING) {
          tree_.haltTree();
        }
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SageBehaviorTreeNode>());
  rclcpp::shutdown();
  return 0;
}