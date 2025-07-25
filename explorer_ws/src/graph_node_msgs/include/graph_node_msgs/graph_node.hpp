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
    /// \param id Integer reference for the node ID
    void setId(int& id);

    /// \brief Sets the 3D position of the node.
    /// \param position Reference to a geometry_msgs::msg::Point representing the node's centroid
    void setPosition(geometry_msgs::msg::Point& position);

    /// \brief Sets the semantic score of the node.
    /// \param score Reference to the score (e.g., derived from color intensity or likelihood)
    void setScore(double& score);

    // === Getters ===

    /// \brief Gets the ID of the node.
    /// \return Integer node ID
    int getId() const;

    /// @brief Gets the GraphNode message representation.
    /// @return graph_node_msgs::msg::GraphNode containing the node's data
    graph_node_msgs::msg::GraphNode getGraphNode() const;

    /// \brief Gets the position of the node.
    /// \return geometry_msgs::msg::Point with the node's position
    geometry_msgs::msg::Point getPosition() const;

    /// \brief Gets the semantic score of the node.
    /// \return Double value representing the score
    double getScore() const;

private:
    // int id = 0;  ///< Unique identifier of the node
    // geometry_msgs::msg::Point position;  ///< 3D position (e.g., centroid) of the node
    // double score = 0.0;  ///< Semantic or relevance score
    graph_node_msgs::msg::GraphNode graphNode;

};

#endif // GRAPHNODE_H
