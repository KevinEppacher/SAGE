#ifndef RELEVANCE_MAP_NODE_HPP
#define RELEVANCE_MAP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <graph_node_msgs/msg/graph_node_array.hpp>
#include <multimodal_query_msgs/msg/semantic_prompt.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "robot.hpp"

class RelevanceMapNode : public rclcpp::Node
{
public:
    explicit RelevanceMapNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub;
    rclcpp::Subscription<graph_node_msgs::msg::GraphNodeArray>::SharedPtr graphNodeSub;
    rclcpp::Subscription<multimodal_query_msgs::msg::SemanticPrompt>::SharedPtr promptSub;

    // Publisher for filtered graph nodes
    rclcpp::Publisher<graph_node_msgs::msg::GraphNodeArray>::SharedPtr graphNodeFilteredPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr relevanceMapPub;

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    // Parameter callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params);

    // Data
    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    sensor_msgs::msg::CameraInfo::SharedPtr camInfo;
    cv::Mat relevanceMap;

    bool mapInitialized = false;

    // Parameters
    double baseIncrementRate;      // α
    double decayFactor;
    double maxRange;
    double fovDeg;
    double relevanceThreshold;
    bool debugMode;
    bool raytracingEnabled;

    std::string inputMapTopic;
    std::string inputGraphTopic;
    std::string outputGraphTopic;
    std::string cameraInfoTopic;

    std::string frameMap;
    std::string frameRobot;

    std::shared_ptr<Robot> robot;

    // Core functions
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void graphNodeCallback(const graph_node_msgs::msg::GraphNodeArray::SharedPtr msg);
    void promptCallback(const multimodal_query_msgs::msg::SemanticPrompt::SharedPtr msg);

    geometry_msgs::msg::PoseStamped::SharedPtr getRobotPose();

    void resizeRelevanceMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void integrateRelevance(const geometry_msgs::msg::Pose &pose);

    cv::Mat computeFovMask(const geometry_msgs::msg::Pose &pose);
    double computeDistanceGaussian(double dist, double sigma) const;
    float getYaw(const geometry_msgs::msg::Pose &pose) const;
    void publishRelevanceMap();

    void resetRelevanceMap();

};

#endif
