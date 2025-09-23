#ifndef GRAPHNODECOLLECTION_H
#define GRAPHNODECOLLECTION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include "graph_node_msgs/msg/graph_node_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rclcpp/clock.hpp"
#include <graph_node_msgs/graph_node.hpp>

/// \class GraphNodeCollection
/// \brief Manages a collection of GraphNodes and handles visualization.
///
/// This class stores a list of GraphNodes, allows manipulation (adding, clearing),
/// normalization of their scores, and publishes them to RViz using MarkerArrays.
/// Used to visually represent semantic clusters or key points in 3D space.
class GraphNodeCollection {
public:
    /// \brief Constructor.
    /// \param node Pointer to the ROS2 node, used for creating publishers and accessing the clock.
    GraphNodeCollection(rclcpp::Node* node);

    /// \brief Add a GraphNode to the collection.
    /// \param node The GraphNode to be added.
    void addNode(const GraphNode& node);

    /// \brief Clear all stored nodes.
    void clear();

    /// \brief Publish all nodes as RViz markers (spheres and score labels).
    ///
    /// Publishes:
    /// - colored spheres indicating the normalized score
    /// - text markers showing the raw score
    /// - a highlighted marker for the highest score
    void publishPosMarkers();

    /// \brief Publish the current collection of GraphNodes as a GraphNodeArray message.
    void publishGraphNodeArray();

    /// \brief Publisher for GraphNodeArray messages
    rclcpp::Publisher<graph_node_msgs::msg::GraphNodeArray>::SharedPtr graphNodePub;

    /// \brief Clear previously published markers in RViz.
    ///
    /// Sends a DELETEALL action to remove all markers from the "cloud_cluster/centroids" namespace.
    void clearMarkers();

    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    rclcpp::Time getTimestamp() const;


    /// \brief Normalize all node scores to sum up to 1.
    ///
    /// The normalization is useful for visual comparison or probabilistic interpretation.
    void normalizeGraphNodes();

    /// \brief Access the current list of nodes.
    /// \return Const reference to the internal vector of GraphNodes.
    const std::vector<GraphNode>& getNodes() const;

    /// \brief Get the frame ID used for published markers.
    /// \return The frame ID string.
    std::string getFrameId() const;

private:
    std::vector<GraphNode> nodes; ///< List of graph nodes managed by this collection
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub; ///< Publisher for marker visualization
    rclcpp::Clock::SharedPtr clock; ///< Clock used for timestamping markers
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub;
    rclcpp::Time latestClock;

    std::string frameId; ///< Frame ID for published markers, set via parameter

};

#endif // GRAPHNODECOLLECTION_H
