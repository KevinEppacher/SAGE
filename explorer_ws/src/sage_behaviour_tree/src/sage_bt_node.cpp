#include <string>
#include <memory>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/behavior_tree.h"

#include <graph_node_msgs/msg/graph_node_array.hpp>

using namespace std::chrono_literals;

// ANSI color codes
#define RESET  "\033[0m"
#define GREEN  "\033[92m"
#define YELLOW "\033[93m"
#define BOLD   "\033[1m"

//////////////////////////////////////////////////////////////////////////////////

class IsDetected : public BT::ConditionNode
{
public:
    IsDetected(const std::string& name,
               const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node_ptr)
        : BT::ConditionNode(name, config), node_ptr_(std::move(node_ptr))
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Initialized", this->name().c_str());

        // Subscribe once to the topic (topic comes from port/blackboard)
        std::string topic;
        getInput<std::string>("detection_graph_node_topic", topic);

        sub_ = node_ptr_->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
            topic, 10,
            [this](const graph_node_msgs::msg::GraphNodeArray::SharedPtr msg) {
                latest_msg_ = *msg;
                received_message_ = true;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("detection_threshold", 0.8, "Detection threshold"),
            BT::InputPort<std::string>("detection_graph_node_topic", "/detection_graph_nodes/graph_nodes",
                                       "GraphNodeArray topic")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!received_message_) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                        "[%s] Waiting for first message...", this->name().c_str());
            return BT::NodeStatus::RUNNING;
        }

        double threshold = 0.8;
        getInput<double>("detection_threshold", threshold);

        double max_score = -1.0;
        for (const auto &n : latest_msg_.nodes) {
            if (n.score > max_score)
                max_score = n.score;
        }

        if (max_score >= threshold) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                        GREEN BOLD "[%s] DETECTED. Highest=%.3f >= %.2f" RESET,
                        this->name().c_str(), max_score, threshold);
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(node_ptr_->get_logger(),
                    YELLOW BOLD "[%s] Not detected. Highest=%.3f < %.2f" RESET,
                    this->name().c_str(), max_score, threshold);
        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<graph_node_msgs::msg::GraphNodeArray>::SharedPtr sub_;
    graph_node_msgs::msg::GraphNodeArray latest_msg_;
    bool received_message_{false};
};

//////////////////////////////////////////////////////////////////////////////////

class SageBehaviorTreeNode : public rclcpp::Node
{
public:
    SageBehaviorTreeNode() : Node("sage_behavior_tree_node")
    {
        // Declare parameters with defaults
        declare_parameter<std::string>("location_file", "");
        declare_parameter<std::string>("tree_xml_file", "");
        declare_parameter<double>("detection.threshold", 0.8);
        declare_parameter<std::string>("detection.graph_node_topic",
                                       "/detection_graph_nodes/graph_nodes");

        location_file_ = get_parameter("location_file").as_string();
        tree_xml_file_ = get_parameter("tree_xml_file").as_string();
        RCLCPP_INFO(get_logger(), "Using location file %s", location_file_.c_str());
    }

    void execute()
    {
        create_behavior_tree();

        timer_ = create_wall_timer(500ms,
                   std::bind(&SageBehaviorTreeNode::update_behavior_tree, this));

        rclcpp::spin(shared_from_this());
        rclcpp::shutdown();
    }

private:
    void create_behavior_tree()
    {
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<IsDetected>("IsDetected", shared_from_this());

        // Seed blackboard from ROS parameters
        auto bb = BT::Blackboard::create();
        bb->set("detection_threshold", get_parameter("detection.threshold").as_double());
        bb->set("detection_graph_node_topic", get_parameter("detection.graph_node_topic").as_string());
        bb->set("location_file", location_file_);

        tree_ = factory.createTreeFromFile(tree_xml_file_, bb);

        // Print tree structure to stdout
        BT::printTreeRecursively(tree_.rootNode());

        publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
    }

    void update_behavior_tree()
    {
        BT::NodeStatus status = tree_.tickOnce();
        if (status == BT::NodeStatus::RUNNING)
            return;

        if (status == BT::NodeStatus::SUCCESS)
            RCLCPP_INFO(get_logger(), "Finished with SUCCESS");
        else if (status == BT::NodeStatus::FAILURE)
            RCLCPP_INFO(get_logger(), "Finished with FAILURE");

        timer_->cancel();
    }

    // Members
    std::string tree_xml_file_;
    std::string location_file_;

    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
};

//////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SageBehaviorTreeNode>();
    node->execute();
    return 0;
}
