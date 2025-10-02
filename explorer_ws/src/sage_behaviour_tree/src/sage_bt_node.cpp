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
#include "behaviortree_cpp/behavior_tree.h"

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

using namespace std::chrono_literals;

class IsDetected : public BT::ConditionNode
{
public:
    IsDetected(const std::string& name,
               const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node_ptr)
        : BT::ConditionNode(name, config), node_ptr_{node_ptr}
    {
        std::cout << "[" << this->name() << "] Initialized" << std::endl;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("threshold", 0.8, "detection threshold"),
            BT::InputPort<std::string>("topic", "/detection_graph_nodes/graph_nodes",
                                       "GraphNodeArray topic")
        };
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "IsDetected ticked");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;
};

//////////////////////////////////////////////////////////////////////////////////

class SageBehaviorTreeNode : public rclcpp::Node {
    public:
        SageBehaviorTreeNode() : Node("autonomy_node") {
            // Read the location file and shuffle it
            this->declare_parameter<std::string>("location_file", default_location_file);
            location_file_ = this->get_parameter("location_file").as_string();

            RCLCPP_INFO(this->get_logger(),"Using location file %s", location_file_.c_str());

            // Declare and get the other node parameters.
            this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
            tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();
            
            this->declare_parameter<std::string>("target_color", "");
            target_color_ = this->get_parameter("target_color").as_string();

            if (target_color_ != "") {
                RCLCPP_INFO(this->get_logger(), "Searching for target color %s...",
                    target_color_.c_str());
            }
        }

        void execute() {
            // Build and initialize the behavior tree based on parameters.
            create_behavior_tree();

            // Create a timer to tick the behavior tree.
            const auto timer_period = 500ms;
            timer_ = this->create_wall_timer(
                timer_period,
                std::bind(&SageBehaviorTreeNode::update_behavior_tree, this));

            rclcpp::spin(shared_from_this());
            rclcpp::shutdown();
        }

        void create_behavior_tree() {
            // Build a behavior tree from XML and set it up for logging
            BT::BehaviorTreeFactory factory;
            factory.registerNodeType<IsDetected>("IsDetected", shared_from_this());

            auto blackboard = BT::Blackboard::create();
            blackboard->set<std::string>("location_file", location_file_);
            tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);
            
            // Set up tree logging to monitor the tree in Groot2.
            // Default ports (1666/1667) are used by the Nav2 behavior tree, so we use another port.
            // NOTE: You must have the PRO version of Groot2 to view live tree updates.
            publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
        }

        void update_behavior_tree() {
            // Tick the behavior tree.
            BT::NodeStatus tree_status = tree_.tickOnce();
            if (tree_status == BT::NodeStatus::RUNNING) {
                return;
            }
            // Cancel the timer if we hit a terminal state.
            if (tree_status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
                timer_->cancel();
            } else if (tree_status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
                timer_->cancel();
            }
        }

        // Configuration parameters.
        std::string tree_xml_file_;
        std::string location_file_;
        std::string target_color_;

        // ROS and BehaviorTree.CPP variables.
        rclcpp::TimerBase::SharedPtr timer_;
        BT::Tree tree_;
        std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
        std::string default_location_file, default_bt_xml_file;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SageBehaviorTreeNode>();
    node->execute();
    return 0;
}
