#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <memory>

template <typename NodeType, typename... Args>
void registerNodeWithArgs(
    BT::BehaviorTreeFactory& factory,
    const std::string& name,
    Args&&... extraArgs)
{
    factory.registerBuilder<NodeType>(
        name,
        [extraArgs...](const std::string& node_name, const BT::NodeConfiguration& config) {
            return std::make_unique<NodeType>(
                node_name, config, extraArgs...);
        });
}
