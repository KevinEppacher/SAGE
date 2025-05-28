#include "frontier.hpp"

Frontier::Frontier(rclcpp::Node* node) : node(node)
{
    markerPub = node->create_publisher<visualization_msgs::msg::Marker>("frontier_markers", 10);
}

Frontier::~Frontier() {}

void Frontier::addPoint(const geometry_msgs::msg::Point& point)
{
    frontier.points.push_back(point);
    publishPoints();
}

void Frontier::setCentroid(const geometry_msgs::msg::Point& centroid)
{
    frontier.centroid = centroid;
    publishCentroid();
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

void Frontier::publishPoints()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node->get_clock()->now();
    marker.ns = "frontier";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    marker.points = frontier.points;

    markerPub->publish(marker);
}

void Frontier::publishCentroid()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node->get_clock()->now();
    marker.ns = "frontier_centroid";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = frontier.centroid;
    marker.pose.orientation.w = 1.0;  // Identitätsrotation

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.9;

    markerPub->publish(marker);
}