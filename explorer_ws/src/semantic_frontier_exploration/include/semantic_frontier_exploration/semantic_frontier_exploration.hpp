#ifndef SEMANTIC_FRONTIER_EXPLORATION_H
#define SEMANTIC_FRONTIER_EXPLORATION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/map_meta_data.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "frontier.hpp"

class SemanticFrontier : public rclcpp::Node {
public:
    SemanticFrontier();

    ~SemanticFrontier();



private:
    //************ Subscribers ************//
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    //************ Publishers ************//

    //************ Timers ************//
    void timerCallback();

    // Miscellaneous Functions
    void getParameters();

    //************ Member Variables ************//
    // Subscriber variables
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occpancygridSub;

    /// \brief Wall timer to trigger periodic processing
    rclcpp::TimerBase::SharedPtr timer;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr occGrid;

    // Class variables
    double publishFrequency;
    std::shared_ptr<Frontier> frontier;


};

#endif // SEMANTIC_FRONTIER_EXPLORATION_H
