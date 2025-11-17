#pragma once
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// ============================ SageBehaviorTreeNode ============================ //

class SageBehaviorTreeNode : public rclcpp::Node
{
public:
    SageBehaviorTreeNode();
    void execute();

private:
    void create_behavior_tree();
    void update_behavior_tree();

    double bt_tick_rate_ms_;
    std::string tree_xml_file_;
    std::string location_file_;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
};