#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <sage_bt_msgs/srv/startup_check.hpp>
#include <sage_bt_msgs/action/execute_prompt.hpp>
#include "sage_behaviour_tree/robot.hpp"

// ============================================================================
// SageBtActionNode
// ============================================================================

class SageBtActionNode : public rclcpp::Node
{
public:
    using ExecutePrompt = sage_bt_msgs::action::ExecutePrompt;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ExecutePrompt>;

    explicit SageBtActionNode();
    void setup_action_server();
    void init_robot();

private:
    // --- Core BT lifecycle ---
    void create_behavior_tree(const std::shared_ptr<GoalHandle> goal_handle);
    BT::NodeStatus run_behavior_tree(const std::shared_ptr<GoalHandle> goal_handle);

    // --- Action callbacks ---
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ExecutePrompt::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute_bt(const std::shared_ptr<GoalHandle> goal_handle);
    void declare_if_not_declared(const std::string& name, const rclcpp::ParameterValue& value);
    void on_startup_request(
        const std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Request> request,
        std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Response> response);
    bool check_required_interfaces(std::stringstream& report);

    // --- Parameters ---
    double bt_tick_rate_ms_;
    std::string tree_xml_file_;
    std::shared_ptr<BT::Blackboard> blackboard_;
    std::string location_file_;

    // --- BT runtime ---
    BT::Tree tree_;
    
    // --- ROS action server ---
    rclcpp_action::Server<ExecutePrompt>::SharedPtr action_server_;
    
    // in class SageBtActionNode
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
    bool groot_initialized_ = false;

    // --- Service for startup checks ---
    rclcpp::Service<sage_bt_msgs::srv::StartupCheck>::SharedPtr startup_service_;
    bool startup_ready_ = false;
    std::vector<std::string> required_topics_;
    std::vector<std::string> required_services_;

    std::unique_ptr<Robot> robot_;

};
