#ifndef FRONTIERCOLLECTION_H
#define FRONTIERCOLLECTION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "frontier_msgs/msg/frontier.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "frontier.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <thread>
#include <queue>


class FrontierCollection {
public:
    FrontierCollection(rclcpp::Node* node);
    ~FrontierCollection();
    nav_msgs::msg::OccupancyGrid detectFrontierGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid);
    std::vector<Frontier> clusterFrontierGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& frontierGrid);
    void publishFrontiers(const std::vector<Frontier>& frontiers);

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr frontierGridPub;

    /// \brief Callback for updating parameters dynamically
    /// \param params Vector of changed parameters
    /// \return SetParametersResult indicating success
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params);

    /// \brief Parameter callback handle (for dynamic reconfiguration)
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontiersPub;
    
    void getParameters();

    /// \brief Euclidean clustering tolerance (in meters)
    double clusterTolerance;

    /// \brief Minimum cluster size (in points)
    int minClusterSize;

    /// \brief Maximum cluster size (in points)
    int maxClusterSize;

    /// \brief Leaf size for voxel grid filter (in meters)
    double voxelLeafSize;

    std::shared_ptr<Frontier> frontier;
    rclcpp::Node* node;
};

#endif // FRONTIERCOLLECTION_H
