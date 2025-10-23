#include "sage_behaviour_tree/core.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SageBehaviorTreeNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    node->execute();
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
