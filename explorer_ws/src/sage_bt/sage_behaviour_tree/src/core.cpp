#include "sage_behaviour_tree/core.hpp"
#include "sage_behaviour_tree/navigation.hpp"
#include "sage_behaviour_tree/detection.hpp"
#include "sage_behaviour_tree/utils.hpp"
#include "sage_behaviour_tree/decorator.hpp"
#include "sage_behaviour_tree/semantic_prompt.hpp"

#include <behaviortree_cpp/xml_parsing.h>

using namespace std::chrono_literals;

SageBehaviorTreeNode::SageBehaviorTreeNode() : Node("sage_behavior_tree_node")
{
    declare_parameter<std::string>("location_file", "");
    declare_parameter<std::string>("tree_xml_file", "");

    location_file_ = get_parameter("location_file").as_string();
    tree_xml_file_ = get_parameter("tree_xml_file").as_string();
}

void SageBehaviorTreeNode::execute()
{
    create_behavior_tree();
    timer_ = create_wall_timer(500ms, std::bind(&SageBehaviorTreeNode::update_behavior_tree, this));
}

void SageBehaviorTreeNode::create_behavior_tree()
{
    BT::BehaviorTreeFactory factory;

    // Register all custom node types
    factory.registerNodeType<IsDetected>("IsDetected", shared_from_this());
    factory.registerNodeType<GoToGraphNode>("GoToGraphNode", shared_from_this());
    factory.registerNodeType<Spin>("Spin", shared_from_this());
    factory.registerNodeType<SetParameterNode>("SetParameterNode", shared_from_this());
    factory.registerNodeType<SeekoutGraphNodes>("SeekoutGraphNodes", shared_from_this());
    factory.registerNodeType<SaveImageAction>("SaveImageAction", shared_from_this());
    factory.registerNodeType<KeepRunningUntilObjectFound>("KeepRunningUntilObjectFound");
    factory.registerNodeType<RealignToObject>("RealignToObject", shared_from_this());
    factory.registerNodeType<CallEmptyService>("CallEmptyService", shared_from_this());
    factory.registerNodeType<PublishSemanticPrompt>("PublishSemanticPrompt", shared_from_this());
    factory.registerNodeType<ForEachEvaluationPrompt>("ForEachEvaluationPrompt", shared_from_this());
    factory.registerNodeType<ApproachPoseAdjustor>("ApproachPoseAdjustor", shared_from_this());
    factory.registerNodeType<SageBtOrchestrator>("SageBtOrchestrator", shared_from_this());

    // === NEW: export TreeNodesModel to XML for Groot 2 ===
    try
    {
        const std::string model_path = "/tmp/sage_nodes_model.xml";

        // This works for both patched v3 and full v4 factories
        std::string xml_content = BT::writeTreeNodesModelXML(factory, /*compact=*/false);

        std::ofstream out(model_path);
        out << xml_content;
        out.close();

        RCLCPP_INFO(get_logger(),
                    "Exported TreeNodesModel XML (%zu bytes) to %s",
                    xml_content.size(), model_path.c_str());
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(get_logger(),
                    "Failed to export TreeNodesModel XML: %s", e.what());
    }

    // === Build runtime tree ===
    auto bb = BT::Blackboard::create();
    bb->set("location_file", location_file_);

    tree_ = factory.createTreeFromFile(tree_xml_file_, bb);

    tree_.rootBlackboard()->debugMessage();
    BT::printTreeRecursively(tree_.rootNode());

    // Connect to Groot 2 live monitor
    publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
}

void SageBehaviorTreeNode::update_behavior_tree()
{
    BT::NodeStatus status = tree_.tickOnce();

    if (status == BT::NodeStatus::RUNNING)
        return;

    RCLCPP_INFO(get_logger(), "Finished with %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");

    timer_->cancel();
    rclcpp::shutdown();
}
