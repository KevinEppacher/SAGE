#include "frontier.hpp"

Frontier::Frontier(rclcpp::Node* node) : node(node)
{

}

Frontier::~Frontier() {}

void Frontier::addPoint(const geometry_msgs::msg::Point& point)
{
    frontier.points.push_back(point);
}

void Frontier::setCentroid(const geometry_msgs::msg::Point& centroid)
{
    frontier.centroid = centroid;
}

void Frontier::setScore(double score)
{
    frontier.score = score;
}

void Frontier::setDistanceToRobot(double distance)
{
    frontier.distance_to_robot = distance;
}

geometry_msgs::msg::Point Frontier::getPointAt(int index) const
{
    if (index < 0 || index >= static_cast<int>(frontier.points.size())) {
        throw std::out_of_range("Index out of range");
    }
    return frontier.points[index];
}

geometry_msgs::msg::Point Frontier::getCentroid() const
{
    return frontier.centroid;
}

double Frontier::getScore() const
{
    return frontier.score;
}

double Frontier::getDistanceToRobot() const
{
    return frontier.distance_to_robot;
}

frontier_msgs::msg::Frontier Frontier::getFrontier() const
{
    return frontier;
}

