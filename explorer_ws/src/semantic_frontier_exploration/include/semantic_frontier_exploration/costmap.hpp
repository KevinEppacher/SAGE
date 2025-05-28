// costmap_publisher.hpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>

class CostmapPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
    CostmapPublisher();

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    void publishCostmap();
};