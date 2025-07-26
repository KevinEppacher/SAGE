#ifndef SEMANTIC_VALUE_MAP_HPP
#define SEMANTIC_VALUE_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "semantic_score.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "color.hpp"
#include <cstdint>
#include <tuple>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>

class SemanticValueMap {
  public:
    explicit SemanticValueMap(rclcpp_lifecycle::LifecycleNode* node);
    ~SemanticValueMap();

    void updateSemanticMap(const SemanticScore& semScore, const geometry_msgs::msg::PoseStamped& pose);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    //************ Lifecycle Callbacks ************//
    bool on_configure();
    bool on_activate();
    bool on_deactivate();
    bool on_cleanup();
    bool on_shutdown();

  private:
    //************ Subscribers ************//
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub;

    //************ Publishers ************//
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr valueMapInfernoPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr valueMapRawPub;


    //************ Timers ************//

    //************ Miscellaneous Functions ************//
    void resizeMapPreservingValues(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    double getHorizontalFOV(const sensor_msgs::msg::CameraInfo::SharedPtr camInfo);
    void publishValueMapRaw();
    void publishValueMapInferno();
    void publish();

    cv::Mat generateTopdownConeMask(
      const geometry_msgs::msg::Pose& pose,
      const nav_msgs::msg::OccupancyGrid& grid,
      float fovDeg,
      float maxRange);
    
    cv::Mat computeConfidenceMap(
      const geometry_msgs::msg::Pose& pose,
      const nav_msgs::msg::OccupancyGrid& grid,
      float cameraFovDeg,
      const cv::Mat& fovMask);

    float getYawAngle(const geometry_msgs::msg::Pose& pose) const;
    float normalizeAngle(float angle) const;



    //************ Member Variables ************//
    rclcpp_lifecycle::LifecycleNode* node;  // pointer to lifecycle node

    nav_msgs::msg::OccupancyGrid::SharedPtr map;

    sensor_msgs::msg::CameraInfo::SharedPtr camInfo;

    cv::Mat valueMap;
    cv::Mat confidenceMap;
    bool mapInitialized = false;
    float maxSemanticScore = 0.0f;

};

#endif // SEMANTIC_VALUE_MAP_HPP
