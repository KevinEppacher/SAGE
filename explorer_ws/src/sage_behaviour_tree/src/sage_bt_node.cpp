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
#include <graph_node_msgs/msg/graph_node.hpp>

using namespace std::chrono_literals;

// ANSI color codes
#define RESET  "\033[0m"
#define GREEN  "\033[92m"
#define YELLOW "\033[93m"
#define BOLD   "\033[1m"


//////////////////////////////////////////////////////////////////////////////////

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/action_node.h"

#include <nav2_msgs/action/spin.hpp>

class Spin360 : public BT::StatefulActionNode
{
public:
    using Spin = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

    Spin360(const std::string& name,
            const BT::NodeConfiguration& config,
            rclcpp::Node::SharedPtr node_ptr)
        : BT::StatefulActionNode(name, config), node_ptr_(std::move(node_ptr))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("spin_angle", 6.28319, "Angle to spin in radians"),
            BT::InputPort<double>("spin_duration", 15.0, "Max spin duration (seconds)")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!node_ptr_) {
            RCLCPP_ERROR(rclcpp::get_logger("Spin360"), "No ROS2 node provided!");
            return BT::NodeStatus::FAILURE;
        }

        // Get inputs
        double spin_angle, spin_duration;
        getInput("spin_angle", spin_angle);
        getInput("spin_duration", spin_duration);

        // Create client
        client_ptr_ = rclcpp_action::create_client<Spin>(node_ptr_, "/spin");
        if (!client_ptr_->wait_for_action_server(1s)) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "[%s] Spin action server not available", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Build goal
        Spin::Goal goal;
        goal.target_yaw = spin_angle;
        goal.time_allowance = rclcpp::Duration::from_seconds(spin_duration);

        using namespace std::placeholders;
        auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
        options.result_callback = std::bind(&Spin360::resultCallback, this, _1);

        done_flag_ = false;
        goal_handle_future_ = client_ptr_->async_send_goal(goal, options);

        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Started spin %.2f rad (%.1f°), duration %.1f s",
                    name().c_str(), spin_angle, spin_angle * 180.0 / M_PI, spin_duration);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (done_flag_) {
            if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Spin completed successfully", name().c_str());
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Spin failed", name().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Spin halted — cancelling action goal", name().c_str());

        // Try to cancel if goal still running
        if (client_ptr_ && goal_handle_future_.valid()) {
            auto goal_handle = goal_handle_future_.get();
            if (goal_handle) {
                client_ptr_->async_cancel_goal(goal_handle);
                RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Cancelled spin action", name().c_str());
            }
        }
    }

private:
    void resultCallback(const GoalHandleSpin::WrappedResult& result)
    {
        done_flag_ = true;
        nav_result_ = result.code;
    }

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<Spin>::SharedPtr client_ptr_;
    std::shared_future<typename GoalHandleSpin::SharedPtr> goal_handle_future_;
    bool done_flag_{false};
    rclcpp_action::ResultCode nav_result_{};
};


//////////////////////////////////////////////////////////////////////////////////


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/action_node.h"   // BT::StatefulActionNode
#include "behaviortree_cpp/basic_types.h"   // Ports

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <graph_node_msgs/msg/graph_node.hpp>   // Your custom GraphNode msg

// Optional if you want to compute orientation from yaw
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GoToGraphNode : public BT::StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoToGraphNode(const std::string& name,
                  const BT::NodeConfiguration& config,
                  rclcpp::Node::SharedPtr node_ptr)
        : BT::StatefulActionNode(name, config), node_ptr_(std::move(node_ptr))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
                "graph_nodes", "GraphNode to navigate to")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!node_ptr_) {
            RCLCPP_ERROR(rclcpp::get_logger("GoToGraphNode"), "No ROS2 node provided!");
            return BT::NodeStatus::FAILURE;
        }

        // Get target node from blackboard
        auto node_res = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_nodes");
        if (!node_res) {
            RCLCPP_WARN(node_ptr_->get_logger(), "[%s] No graph node to go to", this->name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        target_node_ = node_res.value();

        // Create action client
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");

        if (!client_ptr_->wait_for_action_server(1s)) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "[%s] NavigateToPose server not available", this->name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Build goal
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_ptr_->now();
        goal.pose.pose.position = target_node_->position;

        // Orientation = identity quaternion (no rotation), unless you want to compute from GraphNode
        goal.pose.pose.orientation.w = 1.0;

        // Send goal
        using namespace std::placeholders;
        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = std::bind(&GoToGraphNode::resultCallback, this, _1);

        done_flag_ = false;
        client_ptr_->async_send_goal(goal, options);

        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Sent goal to GraphNode ID=%d (x=%.2f, y=%.2f, score=%.2f)",
                    this->name().c_str(), target_node_->id,
                    target_node_->position.x, target_node_->position.y, target_node_->score);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (done_flag_) {
            if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached", this->name().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Failed to reach goal", this->name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Halted", this->name().c_str());
        // Here you could cancel the action goal if needed
    }

private:
    void resultCallback(const GoalHandleNav::WrappedResult& result)
    {
        done_flag_ = true;
        nav_result_ = result.code;
    }

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    std::shared_ptr<graph_node_msgs::msg::GraphNode> target_node_;
    bool done_flag_{false};
    rclcpp_action::ResultCode nav_result_{};
};

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

        // Subscribe once (topic comes from blackboard/port)
        std::string topic;
        getInput<std::string>("detection_graph_node_topic", topic);

        sub_ = node_ptr_->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
            topic, 10,
            [this](const graph_node_msgs::msg::GraphNodeArray::SharedPtr msg) {
                latest_msg_ = msg;
                received_message_ = true;
                missed_ticks_ = 0;  // reset when new message arrives
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("detection_threshold", 0.8, "Detection threshold"),
            BT::InputPort<std::string>("detection_graph_node_topic",
                                       "/detection_graph_nodes/graph_nodes",
                                       "GraphNodeArray topic"),
            BT::InputPort<int>("max_missed_ticks", 10, "Max ticks to wait for a message"),
            BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
                "graph_nodes", "Best GraphNode detected")
        };
    }

    BT::NodeStatus tick() override
    {
        // Case 1: no message yet
        if (!received_message_) {
            missed_ticks_++;
            int max_missed = 10;
            getInput<int>("max_missed_ticks", max_missed);

            if (missed_ticks_ >= max_missed) {
                RCLCPP_WARN(node_ptr_->get_logger(),
                            "[%s] No message received for %d ticks. Returning FAILURE",
                            this->name().c_str(), missed_ticks_);
                missed_ticks_ = 0;
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_ptr_->get_logger(),
                        "[%s] Waiting for first message... (%d/%d)",
                        this->name().c_str(), missed_ticks_, max_missed);
            return BT::NodeStatus::RUNNING;
        }

        // Get threshold from port
        double threshold = 0.8;
        getInput<double>("detection_threshold", threshold);

        // Find best node
        double max_score = -1.0;
        std::shared_ptr<graph_node_msgs::msg::GraphNode> best_node = nullptr;
        for (const auto &n : latest_msg_->nodes) {
            if (n.score > max_score) {
                max_score = n.score;
                best_node = std::make_shared<graph_node_msgs::msg::GraphNode>(n);
            }
        }

        if (!best_node) {
            RCLCPP_WARN(node_ptr_->get_logger(),
                        "[%s] Message contained no nodes. Returning FAILURE",
                        this->name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Publish best node to blackboard
        setOutput("graph_nodes", best_node);

        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Best node ID=%d Score=%.3f (Threshold=%.2f)",
                    this->name().c_str(), best_node->id, best_node->score, threshold);

        return (max_score >= threshold) ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<graph_node_msgs::msg::GraphNodeArray>::SharedPtr sub_;
    graph_node_msgs::msg::GraphNodeArray::SharedPtr latest_msg_;
    bool received_message_{false};
    int missed_ticks_{0};
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
        factory.registerNodeType<GoToGraphNode>("GoToGraphNode", shared_from_this());
        factory.registerNodeType<Spin360>("Spin360", shared_from_this());

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
