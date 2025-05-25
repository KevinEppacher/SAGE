#ifndef FRONTIER_H
#define FRONTIER_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "frontier_msgs/msg/frontier.hpp"

class Frontier {
public:
    Frontier(rclcpp::Node* node);  // Pass pointer to node
    ~Frontier();

private:
    rclcpp::Node* node_;  // Node handle
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    frontier_msgs::msg::Frontier frontier;
};

#endif // FRONTIER_H
