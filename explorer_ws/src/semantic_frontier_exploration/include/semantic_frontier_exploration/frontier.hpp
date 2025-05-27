#ifndef FRONTIER_H
#define FRONTIER_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "frontier_msgs/msg/frontier.hpp"

class Frontier {
public:
    Frontier(rclcpp::Node* node);  // Pass pointer to node
    ~Frontier();

    void addPoint(const geometry_msgs::msg::Point& point);
    void setCentroid(const geometry_msgs::msg::Point& centroid);
    void setScore(double score);
    void setDistanceToRobot(double distance);
    geometry_msgs::msg::Point getPointAt(int index) const;
    geometry_msgs::msg::Point getCentroid() const;
    double getScore() const;
    double getDistanceToRobot() const;
    frontier_msgs::msg::Frontier getFrontier() const;

private:
    rclcpp::Node* node;  // Node handle
    frontier_msgs::msg::Frontier frontier;
};

#endif // FRONTIER_H
