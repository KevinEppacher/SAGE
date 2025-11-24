#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Robot
{
public:
    explicit Robot(rclcpp::Node *node);

    geometry_msgs::msg::PoseStamped::SharedPtr getPose();

    // Optional helper: compute horizontal FOV from CameraInfo
    double computeHorizontalFov(const sensor_msgs::msg::CameraInfo &info);

private:
    rclcpp::Node *node;

    std::string frameMap;
    std::string frameRobot;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
};

#endif
