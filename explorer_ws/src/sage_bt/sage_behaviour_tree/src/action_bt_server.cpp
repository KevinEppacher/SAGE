#include "sage_behaviour_tree/action_bt_server.hpp"

#include "sage_behaviour_tree/navigation.hpp"
#include "sage_behaviour_tree/detection.hpp"
#include "sage_behaviour_tree/utils.hpp"
#include "sage_behaviour_tree/decorator.hpp"
#include "sage_behaviour_tree/semantic_prompt.hpp"
#include "sage_behaviour_tree/observe_graph_nodes.hpp"

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

    // --- Declare startup requirements ---
    declare_if_not_declared("startup.required_topics",
        rclcpp::ParameterValue(std::vector<std::string>{}));
    declare_if_not_declared("startup.required_services",
        rclcpp::ParameterValue(std::vector<std::string>{}));

    required_topics_   = get_parameter("startup.required_topics").as_string_array();
    required_services_ = get_parameter("startup.required_services").as_string_array();

    // Log parameters
    RCLCPP_INFO(get_logger(), "Behavior Tree XML file: %s", tree_xml_file_.c_str());
    RCLCPP_INFO(get_logger(), "BT tick rate (ms): %.1f", bt_tick_rate_ms_);
    RCLCPP_INFO(get_logger(), "Location file: %s", location_file_.c_str());
    RCLCPP_INFO(get_logger(), "Required topics for startup check:");
    for (const auto& topic : required_topics_)
        RCLCPP_INFO(get_logger(), "  - %s", topic.c_str());
    RCLCPP_INFO(get_logger(), "Required services for startup check:");
    for (const auto& service : required_services_)
        RCLCPP_INFO(get_logger(), "  - %s", service.c_str());

    // --- StartupCheck service ---
    startup_service_ = create_service<sage_bt_msgs::srv::StartupCheck>(
        "startup_check",
        std::bind(&SageBtActionNode::on_startup_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "StartupCheck service '/startup_check' active");

}

void SageBtActionNode::declare_if_not_declared(
    const std::string& name,
    const rclcpp::ParameterValue& value)
{
    if (!has_parameter(name))
        declare_parameter(name, value);
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
    if (!startup_ready_)
    {
        RCLCPP_WARN(get_logger(),
            "Rejecting goal '%s' → StartupCheck not passed yet.",
            goal->prompt.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

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
    auto self = shared_from_this();
    std::thread([this, self, goal_handle]() {
        this->execute_bt(goal_handle);
    }).detach();
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
    factory.registerNodeType<ObserveGraphNodes>("ObserveGraphNodes", shared_from_this());

    // --- Export XML model for Groot2 ---
    try 
    {
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
    blackboard_->set("object_found", false);
    blackboard_->set("any_exploration_nodes", true);
    blackboard_->set<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
        "detected_graph_node", std::make_shared<graph_node_msgs::msg::GraphNode>());

    tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard_);

    // optional debugging
    RCLCPP_INFO(get_logger(), "Blackboard before execution:");
    blackboard_->debugMessage();

    BT::printTreeRecursively(tree_.rootNode());

    // before creating new Groot publisher
    if (publisher_ptr_) 
    {
        publisher_ptr_.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
}

void SageBtActionNode::execute_bt(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecutePrompt::Feedback>();
    auto result   = std::make_shared<ExecutePrompt::Result>();

    RCLCPP_INFO(get_logger(),
        "Starting Behavior Tree execution with prompt: %s (timeout: %.1f min)",
        goal->prompt.c_str(), goal->timeout);

    if (!robot_)
        robot_ = std::make_unique<Robot>(shared_from_this());

    robot_->resetPath();          // start a fresh path recording
    robot_->recordCurrentPose();  // record initial pose

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("text_query", goal->prompt);
    blackboard_->set("zero_shot_query", goal->zero_shot_prompt);
    blackboard_->set("image_path", goal->save_directory);
    blackboard_->set<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
        "detected_graph_node", std::make_shared<graph_node_msgs::msg::GraphNode>());

    create_behavior_tree(goal_handle);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Rate rate(1000.0 / bt_tick_rate_ms_);

    // ----------------------------------------------------
    // Timeout (steady clock)
    // ----------------------------------------------------
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    const double timeout_sec = goal->timeout * 60.0;
    const rclcpp::Time start_time = steady_clock.now();

    // ----------------------------------------------------
    // Main execution loop
    // ----------------------------------------------------
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        const double elapsed = (steady_clock.now() - start_time).seconds();

        if (timeout_sec > 0.0 && elapsed >= timeout_sec)
        {
            RCLCPP_ERROR(get_logger(),
                "BT execution TIMEOUT after %.1f sec (limit %.1f sec)",
                elapsed, timeout_sec);
            status = BT::NodeStatus::FAILURE;
            break;
        }

        // Client cancel
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(get_logger(), "Goal canceled by client");
            result->result = false;
            result->confidence_score = 0.0f;
            if (rclcpp::ok() && goal_handle->is_active())
                goal_handle->canceled(result);
            tree_.haltTree();
            return;
        }

        // Tick BT once and record current pose
        status = tree_.tickOnce();
        robot_->recordCurrentPose();

        feedback->active_node = "BT_Tick";
        feedback->status = (status == BT::NodeStatus::RUNNING);
        feedback->log = "Ticking behavior tree...";
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }

    // ----------------------------------------------------
    // Retrieve detected node info from blackboard
    // ----------------------------------------------------
    std::shared_ptr<graph_node_msgs::msg::GraphNode> detected_node;
    if (blackboard_->get("detected_graph_node", detected_node) && detected_node)
    {
        RCLCPP_INFO(get_logger(),
            "Retrieved detected_graph_node → id=%d, score=%.2f, pos(%.2f, %.2f, %.2f), relevance=%.2f",
            detected_node->id,
            detected_node->score,
            detected_node->position.x,
            detected_node->position.y,
            detected_node->position.z,
            detected_node->relevance);

        result->confidence_score = static_cast<float>(detected_node->score);
        result->end_pose.position = detected_node->position;
    }
    else
    {
        result->confidence_score = 0.0f;
        RCLCPP_WARN(get_logger(), "No detected_graph_node found → confidence=0.0");
    }

    // ----------------------------------------------------
    // Add robot trajectory data to the result
    // ----------------------------------------------------
    result->start_pose = robot_->getStartPose();
    result->end_pose   = robot_->getEndPose();
    result->accumulated_path = robot_->getAccumulatedPath();

    // ----------------------------------------------------
    // Compute and store total execution time
    // ----------------------------------------------------
    const double total_elapsed_sec = (steady_clock.now() - start_time).seconds();
    result->total_time = static_cast<float>(total_elapsed_sec);
    RCLCPP_INFO(get_logger(), "Total BT execution time: %.2f seconds", total_elapsed_sec);

    // ----------------------------------------------------
    // Publish final result
    // ----------------------------------------------------
    result->result = (status == BT::NodeStatus::SUCCESS);

    if (rclcpp::ok() && goal_handle->is_active())
    {
        if (status == BT::NodeStatus::SUCCESS)
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "BT execution SUCCESS");
        }
        else
        {
            goal_handle->abort(result);
            RCLCPP_WARN(get_logger(), "BT execution FAILURE or TIMEOUT");
        }
    }
}

void SageBtActionNode::on_startup_request(
    const std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Request>,
    std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Response> response)
{
    std::stringstream report;
    bool all_ok = check_required_interfaces(report);

    response->ready = all_ok;
    response->report = report.str();

    if (all_ok)
    {
        startup_ready_ = true;
        RCLCPP_INFO(get_logger(),
            "[StartupCheck] SUCCESS: all required topics/services found.\n%s",
            response->report.c_str());
    }
    else
    {
        RCLCPP_WARN(get_logger(),
            "[StartupCheck] FAILED:\n%s", response->report.c_str());
    }
}

bool SageBtActionNode::check_required_interfaces(std::stringstream& report)
{
    bool all_ok = true;

    auto available_topics = get_topic_names_and_types();
    auto available_services = get_service_names_and_types();

    for (const auto& topic : required_topics_)
    {
        if (available_topics.count(topic) == 0)
        {
            all_ok = false;
            report << "Missing topic: " << topic << "\n";
        }
        else
        {
            report << "✓ Found topic: " << topic << "\n";
        }
    }

    for (const auto& srv : required_services_)
    {
        if (available_services.count(srv) == 0)
        {
            all_ok = false;
            report << "Missing service: " << srv << "\n";
        }
        else
        {
            report << "✓ Found service: " << srv << "\n";
        }
    }

    return all_ok;
}