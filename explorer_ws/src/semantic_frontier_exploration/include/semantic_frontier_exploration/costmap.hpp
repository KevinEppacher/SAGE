#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class Costmap : public rclcpp::Node {
public:
    Costmap();
    ~Costmap();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timerCallback();
    void updateOccupancyGrid();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    float resolution_;
    int width_, height_;
    float origin_x_, origin_y_;
};
