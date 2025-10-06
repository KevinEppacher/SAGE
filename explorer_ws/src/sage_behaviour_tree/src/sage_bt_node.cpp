#include "sage_behaviour_tree/core.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SageBehaviorTreeNode>();
    node->execute();
    rclcpp::shutdown();
    return 0;
}
