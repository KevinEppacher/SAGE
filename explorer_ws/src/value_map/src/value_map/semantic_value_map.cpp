#include "semantic_value_map.hpp"

SemanticValueMap::SemanticValueMap(rclcpp_lifecycle::LifecycleNode* node)
: node(node)
{

}

SemanticValueMap::~SemanticValueMap() 
{

}

bool SemanticValueMap::on_configure() 
{
    RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Configuring...");
    // Init subscribers/publishers
    return true;
}

bool SemanticValueMap::on_activate() 
{
    RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Activating...");
    // Activate lifecycle publishers (e.g. my_pub_->on_activate())
    return true;
}

bool SemanticValueMap::on_deactivate() 
{
    RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Deactivating...");
    return true;
}

bool SemanticValueMap::on_cleanup() 
{
    RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Cleaning up...");
    return true;
}

bool SemanticValueMap::on_shutdown() 
{
    RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Shutting down...");
    return true;
}

void SemanticValueMap::updateSemanticMap() 
{

}