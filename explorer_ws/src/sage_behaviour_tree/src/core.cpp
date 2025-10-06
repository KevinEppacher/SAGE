#include "sage_behaviour_tree/core.hpp"
#include "sage_behaviour_tree/navigation.hpp"
#include "sage_behaviour_tree/detection.hpp"
#include "sage_behaviour_tree/utils.hpp"

using namespace std::chrono_literals;

SageBehaviorTreeNode::SageBehaviorTreeNode() : Node("sage_behavior_tree_node")
{
    declare_parameter<std::string>("location_file", "");
    declare_parameter<std::string>("tree_xml_file", "");
    declare_parameter<double>("detection.threshold", 0.8);
    declare_parameter<std::string>("detection.graph_node_topic",
                                   "/detection_graph_nodes/graph_nodes");

    location_file_ = get_parameter("location_file").as_string();
    tree_xml_file_ = get_parameter("tree_xml_file").as_string();
}

void SageBehaviorTreeNode::execute()
{
    create_behavior_tree();
    timer_ = create_wall_timer(500ms, std::bind(&SageBehaviorTreeNode::update_behavior_tree, this));
    rclcpp::spin(shared_from_this());
    rclcpp::shutdown();
}

void SageBehaviorTreeNode::create_behavior_tree()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<IsDetected>("IsDetected", shared_from_this());
    factory.registerNodeType<GoToGraphNode>("GoToGraphNode", shared_from_this());
    factory.registerNodeType<Spin360>("Spin360", shared_from_this());
    factory.registerNodeType<SetParameterNode>("SetParameterNode", shared_from_this());

    auto bb = BT::Blackboard::create();
    bb->set("detection_threshold", get_parameter("detection.threshold").as_double());
    bb->set("detection_graph_node_topic", get_parameter("detection.graph_node_topic").as_string());
    bb->set("location_file", location_file_);

    tree_ = factory.createTreeFromFile(tree_xml_file_, bb);
    tree_.rootBlackboard()->debugMessage();
    BT::printTreeRecursively(tree_.rootNode());
    publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
}

void SageBehaviorTreeNode::update_behavior_tree()
{
    BT::NodeStatus status = tree_.tickOnce();
    if (status == BT::NodeStatus::RUNNING) return;

    RCLCPP_INFO(get_logger(), "Finished with %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
    timer_->cancel();
}
