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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <multimodal_query_msgs/msg/semantic_prompt.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>

class SemanticValueMap {
  public:
    explicit SemanticValueMap(rclcpp_lifecycle::LifecycleNode* node);
    ~SemanticValueMap();

    void updateSemanticMap(const SemanticScore& semScore, const geometry_msgs::msg::PoseStamped& pose);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void clearValueMap();

    //************ Lifecycle Callbacks ************//
    bool on_configure();
    bool on_activate();
    bool on_deactivate();
    bool on_cleanup();
    bool on_shutdown();

  private:
    //************ Parameter Callback ************//
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle;
    
    //************ Subscribers ************//
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub;

    //************ Publishers ************//
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr valueMapInfernoPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr valueMapRawPub;

    //************ Services ************//
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearValueMapService;
    void handleClearValueMap(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
        
        //************ Miscellaneous Functions ************//
    void resizeMapPreservingValues(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    double getHorizontalFOV(const sensor_msgs::msg::CameraInfo::SharedPtr camInfo);
    void publishValueMapRaw();
    void publishValueMapInferno();

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
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameterCallbackHandle;
    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    sensor_msgs::msg::CameraInfo::SharedPtr camInfo;

    cv::Mat valueMap;
    cv::Mat confidenceMap;
    bool mapInitialized = false;
    float maxSemanticScore = 1.0f;
    float confidenceSharpness = 10.0f;  // exponent = 2.0 --> Standard, exponent > 2.0 --> Sharper confidence decay, exponent < 2.0 --> Smoother confidence decay
    double decayFactor = 0.99;  // decay factor for previous values
    float maxRange = 10.0;  // Default maximum range for semantic mapping
    std::string mapTopic = "/map";  // Topic for the occupancy grid map
    std::string cameraInfoTopic = "/camera_info";  // Topic for the camera info
    bool visualizeConfidenceMap = false;  // Flag to visualize the confidence map
    bool confidenceWindowOpen = false;
    float updateGain = 0.3f;  // Default update gain
    std::string clearValueMapServiceName = "/clear_value_map";
    bool useCameraInfoFov = true;  // Flag to use camera info FOV
    double fovDeg = 30.0;  // Default FOV in degrees if not

    geometry_msgs::msg::Pose lastPose;
    bool lastPoseValid = false;
    double moveThreshold = 0.05;  // meters
    float distanceSigma = 3.0f;  // in meters
    float distanceSigmaPx = 0.0f;  // in pixels
};

#endif // SEMANTIC_VALUE_MAP_HPP
