#ifndef SEMANTIC_VALUE_MAP_HPP
#define SEMANTIC_VALUE_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class SemanticValueMap {
    public:
        explicit SemanticValueMap(rclcpp_lifecycle::LifecycleNode* node);
        ~SemanticValueMap();

        bool on_configure();
        bool on_activate();
        bool on_deactivate();
        bool on_cleanup();
        bool on_shutdown();

    private:
    
    //************ Subscribers ************//
    
    //************ Publishers ************//
    
    //************ Timers ************//
    
    //************ Member Variables ************//
    rclcpp_lifecycle::LifecycleNode* node;  // pointer to lifecycle node
};

#endif // SEMANTIC_VALUE_MAP_HPP
