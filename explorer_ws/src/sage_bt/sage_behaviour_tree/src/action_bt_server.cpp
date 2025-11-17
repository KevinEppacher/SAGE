#include "sage_behaviour_tree/action_bt_server.hpp"

#include "sage_behaviour_tree/navigation.hpp"
#include "sage_behaviour_tree/detection.hpp"
#include "sage_behaviour_tree/utils.hpp"
#include "sage_behaviour_tree/decorator.hpp"
#include "sage_behaviour_tree/semantic_prompt.hpp"

#include <behaviortree_cpp/xml_parsing.h>
#include <fstream>

using namespace std::chrono_literals;

// ============================================================================
// Constructor
// ============================================================================

SageBtActionNode::SageBtActionNode()
    : Node("sage_behavior_tree_node")
{
    // --- Parameters ---
    declare_parameter<std::string>("location_file", "");
    declare_parameter<std::string>("tree_xml_file", "");
    declare_parameter<double>("bt_tick_rate_ms", 1000.0);

    location_file_   = get_parameter("location_file").as_string();
    tree_xml_file_   = get_parameter("tree_xml_file").as_string();
    bt_tick_rate_ms_ = get_parameter("bt_tick_rate_ms").as_double();

}

void SageBtActionNode::setup_action_server()
{
    action_server_ = rclcpp_action::create_server<ExecutePrompt>(
        shared_from_this(),
        "execute_prompt",
        std::bind(&SageBtActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SageBtActionNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&SageBtActionNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Action server 'execute_prompt' ready");
}


// ============================================================================
// Action callbacks
// ============================================================================

rclcpp_action::GoalResponse SageBtActionNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const ExecutePrompt::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal: %s", goal->prompt.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SageBtActionNode::handle_cancel(
    const std::shared_ptr<GoalHandle>)
{
    RCLCPP_WARN(get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SageBtActionNode::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    // run BT asynchronously
    std::thread{&SageBtActionNode::execute_bt, this, goal_handle}.detach();
}

// ============================================================================
// BT creation and execution
// ============================================================================

void SageBtActionNode::create_behavior_tree(const std::shared_ptr<GoalHandle> goal_handle)
{
    BT::BehaviorTreeFactory factory;

    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecutePrompt::Feedback>();
    auto result   = std::make_shared<ExecutePrompt::Result>();

    // --- Node registration ---
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
    factory.registerNodeType<ApproachPoseAdjustor>("ApproachPoseAdjustor", shared_from_this());

    // --- Export XML model for Groot2 ---
    try {
        const std::string model_path = "/tmp/sage_nodes_model.xml";
        std::string xml_content = BT::writeTreeNodesModelXML(factory);
        std::ofstream(model_path) << xml_content;
        RCLCPP_INFO(get_logger(), "Exported BT node model (%zu bytes)", xml_content.size());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "Failed to export node model");
    }

    if (!blackboard_)
        blackboard_ = BT::Blackboard::create();

    blackboard_->set("location_file", location_file_);
    blackboard_->set("text_query", goal->prompt);
    blackboard_->set("image_path", goal->save_directory);
    blackboard_->set<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
        "detected_graph_node", std::make_shared<graph_node_msgs::msg::GraphNode>());

    tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard_);

    // optional debugging
    RCLCPP_INFO(get_logger(), "Blackboard before execution:");
    blackboard_->debugMessage();

    BT::printTreeRecursively(tree_.rootNode());

    // // Create or update Groot2Publisher safely
    // if (!groot_initialized_) {
    //     publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
    //     groot_initialized_ = true;
    //     RCLCPP_INFO(get_logger(), "Initialized Groot2Publisher on port 1668");
    // } else {
    //     publisher_ptr_->setTree(tree_);
    //     RCLCPP_INFO(get_logger(), "Reusing existing Groot2Publisher instance");
    // }
    // before creating new Groot publisher
    if (publisher_ptr_) 
    {
        publisher_ptr_.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
}

BT::NodeStatus SageBtActionNode::run_behavior_tree(const std::shared_ptr<GoalHandle> goal_handle)
{
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Rate rate(1000.0 / bt_tick_rate_ms_);

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        // Check if goal has been canceled
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(get_logger(), "BT execution interrupted by cancel request");
            break;
        }

        status = tree_.tickOnce();
        rate.sleep();
    }

    return status;
}


void SageBtActionNode::execute_bt(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecutePrompt::Feedback>();
    auto result   = std::make_shared<ExecutePrompt::Result>();

    RCLCPP_INFO(get_logger(), "Starting Behavior Tree execution with prompt: %s",
                goal->prompt.c_str());

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("text_query", goal->prompt);
    blackboard_->set("image_path", goal->save_directory);
    blackboard_->set<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
        "detected_graph_node", std::make_shared<graph_node_msgs::msg::GraphNode>());

    create_behavior_tree(goal_handle);   // use blackboard_ directly inside
    BT::NodeStatus status = run_behavior_tree(goal_handle);

    if (goal_handle->is_canceling())
    {
        result->result = false;
        goal_handle->canceled(result);
        RCLCPP_WARN(get_logger(), "Goal canceled");
        return;
    }

    result->result = (status == BT::NodeStatus::SUCCESS);
    result->confidence_score = 1.0f;

    if (status == BT::NodeStatus::SUCCESS)
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "BT execution SUCCESS");
    }
    else
    {
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "BT execution FAILURE");
    }
}
