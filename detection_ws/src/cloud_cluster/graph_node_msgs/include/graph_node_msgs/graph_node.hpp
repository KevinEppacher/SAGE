#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "graph_node_msgs/msg/graph_node.hpp"

/// \class GraphNode
/// \brief Represents a node in the graph containing spatial and semantic information.
///
/// Each GraphNode holds an identifier, a 3D position, and a semantic score
/// that can be used for tasks like clustering visualization, path planning,
/// or relevance ranking.
class GraphNode {
public:
    /// \brief Default constructor.
    GraphNode();

    /// \brief Default destructor.
    ~GraphNode();

    // === Setters ===

    /// \brief Sets the ID of the node.
    /// \param id Integer node ID
    void setId(int id);

    /// \brief Sets the 3D position of the node.
    /// \param position geometry_msgs::msg::Point representing the node's centroid
    void setPosition(const geometry_msgs::msg::Point& position);

    /// \brief Sets the semantic score of the node.
    /// \param score Double value representing the score
    void setScore(double score);

    /// \brief Sets whether the node has been observed.
    /// \param is_observed Boolean indicating observation status
    bool getIsObserved() const;

    /// \brief Sets whether the node has been observed.
    /// \param is_observed Boolean indicating observation status
    void setIsObserved(bool is_observed);

    // === Getters ===
    int getId() const;
    geometry_msgs::msg::Point getPosition() const;
    double getScore() const;

    /// \brief Gets the full GraphNode ROS message.
    graph_node_msgs::msg::GraphNode getGraphNode() const;

private:
    graph_node_msgs::msg::GraphNode graphNode;
};

#endif // GRAPHNODE_H
