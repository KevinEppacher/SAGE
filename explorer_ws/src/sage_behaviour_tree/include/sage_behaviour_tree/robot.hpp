#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <graph_node_msgs/msg/graph_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

class Robot
{
    public:
        Robot(rclcpp::Node::SharedPtr nodePtr);
        ~Robot();

        bool getPose(geometry_msgs::msg::Pose & out_pose,
                         const std::string & map_frame = "map",
                         const std::string & robot_frame = "base_link");

    private:
        rclcpp::Node::SharedPtr nodePtr;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
};