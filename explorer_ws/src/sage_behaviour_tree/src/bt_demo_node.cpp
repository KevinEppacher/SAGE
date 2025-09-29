#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <chrono>
#include <thread>
#include <memory>

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
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_demo_node");
  node->declare_parameter<int>("zmq_pub", 1666);
  node->declare_parameter<int>("zmq_srv", 1667);
  node->declare_parameter<double>("tick_hz", 20.0);
  const int zmq_pub = node->get_parameter("zmq_pub").as_int();
  const int zmq_srv = node->get_parameter("zmq_srv").as_int();
  const double hz = node->get_parameter("tick_hz").as_double();

  static const char* XML = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <Init/>
      <WaitMs ms="500"/>
      <Work/>
    </Sequence>
  </BehaviorTree>
</root>
)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Init>("Init");
  factory.registerNodeType<Work>("Work");
  factory.registerNodeType<WaitMs>("WaitMs");

    auto tree = factory.createTreeFromText(XML);

    // one publisher only
    BT::PublisherZMQ publisher(tree, 25, zmq_pub, zmq_srv);

    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
    BT::NodeStatus status = tree.tickRoot();

    if (status != BT::NodeStatus::RUNNING) {
        tree.rootNode()->halt();            // reset to IDLE and run again
    }

    rclcpp::spin_some(node);
    rate.sleep();
    }
  rclcpp::shutdown();
  return 0;
}
