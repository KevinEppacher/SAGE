#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Robot
{
public:
    explicit Robot(rclcpp::Node::SharedPtr nodePtr);
    ~Robot() = default;

    // Return true if lookup successful
    bool getPose(geometry_msgs::msg::Pose & out_pose,
                 const std::string & map_frame = "map",
                 const std::string & robot_frame = "base_link");

private:
    rclcpp::Node::SharedPtr nodePtr;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    tf2_ros::TransformListener tfListener;  // not shared_ptr
};