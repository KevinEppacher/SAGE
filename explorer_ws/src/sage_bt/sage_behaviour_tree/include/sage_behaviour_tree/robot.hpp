#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <graph_node_msgs/msg/graph_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/utils.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

class Robot
{
public:
    explicit Robot(rclcpp::Node::SharedPtr node);
    ~Robot() = default;

    bool getPose(geometry_msgs::msg::Pose& outPose,
                 const std::string& mapFrame = "map",
                 const std::string& robotFrame = "base_link");

    bool spin(double yaw, double durationSec);
    void cancelSpin();
    bool isSpinDone() const;
    bool spinRelativeTo(const std::string& frame, double targetYawAbs, double durationSec);
    rclcpp_action::ResultCode getSpinResult() const;

    void cancelNavigationGoals();
    void publishGoalToTarget(const graph_node_msgs::msg::GraphNode& node,
                             const std::string& goalTopic = "/goal_pose",
                             const std::string& frame = "map");
    bool isHalted();
    void recordCurrentPose();
    nav_msgs::msg::Path getAccumulatedPath(const std::string& reference_frame_id = "map") const;
    geometry_msgs::msg::Pose getStartPose() const { return startPose; }
    geometry_msgs::msg::Pose getEndPose() const { return endPose; }
    void resetPath();

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> getGlobalCostmap();

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    tf2_ros::TransformListener tfListener;

    using NavSpin = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<NavSpin>;

    rclcpp_action::Client<NavSpin>::SharedPtr spinClient;
    std::shared_future<typename GoalHandleSpin::SharedPtr> goalHandleFuture;
    rclcpp_action::ResultCode spinResult{rclcpp_action::ResultCode::UNKNOWN};

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapSub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> latestCostmap;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> latestMap;

    bool spinDone{false};
    bool halted{false};
    rclcpp::Time haltedTime{};

    std::vector<geometry_msgs::msg::PoseStamped> pathHistory;
    geometry_msgs::msg::Pose startPose;
    geometry_msgs::msg::Pose endPose;
    bool startPoseSet{false};
};