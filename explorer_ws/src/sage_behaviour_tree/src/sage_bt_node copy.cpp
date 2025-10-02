#include <string>
#include <memory>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include <graph_node_msgs/msg/graph_node_array.hpp>

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[91m"
#define GREEN "\033[92m"
#define YELLOW "\033[93m"
#define BLUE "\033[94m"
#define BOLD "\033[1m"

class IsDetected : public BT::ConditionNode
{
public:
    // Use the STANDARD constructor signature required by BT.cpp
    IsDetected(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
        // Get ROS node from blackboard instead of constructor
        RCLCPP_INFO(node->get_logger(), "IsDetected node created");
    }

    // Declare blackboard ports here
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("threshold", 0.8, "detection threshold"),
            BT::InputPort<std::string>("topic", "/detection_graph_nodes/graph_nodes",
                                       "GraphNodeArray topic")};
    }

    // Main evaluation
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node->get_logger(), "IsDetected ticked");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node;
};

//////////////////////////////////////////////////////////////////////////////////

class SageBehaviorTreeNode : public LifecycleNode
{
public:
    SageBehaviorTreeNode()
        : LifecycleNode("sage_behavior_tree_node")
    {
        declareAndReadParameters();
    }

    // ===== Lifecycle Callbacks =====
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        try
        {
            resolveXmlPath();
            buildTree();
            startGrootPublisher();
            RCLCPP_INFO(get_logger(), "Configured. XML: %s | Tick: %d ms | Groot2: %d",
                        xml_path_.c_str(), tick_period_ms_, groot_port_);
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "on_configure: %s", e.what());
            return CallbackReturn::FAILURE;
        }
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        startTickTimer();
        RCLCPP_INFO(get_logger(), "Activated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        stopTickTimer();
        safeHaltTree();
        RCLCPP_INFO(get_logger(), "Deactivated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        stopTickTimer();
        safeHaltTree();
        publisher_.reset();
        tree_ = BT::Tree{};
        RCLCPP_INFO(get_logger(), "Cleaned up.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        stopTickTimer();
        safeHaltTree();
        publisher_.reset();
        RCLCPP_INFO(get_logger(), "Shut down.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State &)
    {
        stopTickTimer();
        safeHaltTree();
        publisher_.reset();
        RCLCPP_ERROR(get_logger(), "Error state.");
        return CallbackReturn::SUCCESS;
    }

private:
    // ===== Parameters =====
    void declareAndReadParameters()
    {
        xml_path_ = this->declare_parameter<std::string>("tree_xml_file", "");
        location_file_ = this->declare_parameter<std::string>("location_file", "");
        tick_period_ms_ = this->declare_parameter<int>("tick_period_ms", 50);
        groot_port_ = this->declare_parameter<int>("groot_port", 1668);
    }

    void resolveXmlPath()
    {
        if (!xml_path_.empty())
            return;
        const auto share = ament_index_cpp::get_package_share_directory("sage_behaviour_tree");
        xml_path_ = share + std::string("/bt_xml/demo.xml");
    }

    // ===== BehaviorTree setup =====
    void registerCustomNodes(BT::BehaviorTreeFactory &factory)
    {
        // Register your custom BT nodes here
        factory.registerNodeType<IsDetected>("IsDetected", shared_from_this());
    }

    void buildTree()
    {
        BT::BehaviorTreeFactory factory;
        registerCustomNodes(factory);

        auto blackboard = BT::Blackboard::create();
        blackboard->set<std::string>("location_file", location_file_);

        tree_ = factory.createTreeFromFile(xml_path_, blackboard);
        BT::printTreeRecursively(tree_.rootNode());
    }

    void startGrootPublisher()
    {
        publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, static_cast<uint16_t>(groot_port_));
    }

    // ===== Timer / Ticking =====
    void startTickTimer()
    {
        if (timer_)
            return;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(tick_period_ms_),
            std::bind(&SageBehaviorTreeNode::onTickTimer, this));
    }

    void stopTickTimer()
    {
        if (!timer_)
            return;
        timer_->cancel();
        timer_.reset();
    }

    void onTickTimer()
    {
        const BT::NodeStatus status = tree_.tickOnce();
        if (status != BT::NodeStatus::RUNNING)
        {
            safeHaltTree();
        }
    }

    void safeHaltTree()
    {
        if (tree_.rootNode())
        {
            tree_.haltTree();
        }
    }

    // ===== Members =====
    std::string xml_path_;
    std::string location_file_;
    int tick_period_ms_{50};
    int groot_port_{1668};

    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SageBehaviorTreeNode>();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());

    // Autostart here: configure -> activate
    (void)node->configure();
    (void)node->activate();

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
