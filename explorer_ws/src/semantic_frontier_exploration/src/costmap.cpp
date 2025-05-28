// costmap_publisher.cpp

#include "costmap.hpp"
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_publisher.hpp>
#include <nav2_util/lifecycle_utils.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

void convertCostmapToOccupancyGrid(
    const nav2_costmap_2d::Costmap2D* costmap,
    nav_msgs::msg::OccupancyGrid & msg,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
{
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;

    msg.info.resolution = costmap->getResolution();
    msg.info.width = costmap->getSizeInCellsX();
    msg.info.height = costmap->getSizeInCellsY();
    msg.info.origin.position.x = costmap->getOriginX();
    msg.info.origin.position.y = costmap->getOriginY();
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(msg.info.width * msg.info.height);

    for (unsigned int y = 0; y < msg.info.height; ++y) {
        for (unsigned int x = 0; x < msg.info.width; ++x) {
            unsigned char cost = costmap->getCost(x, y);
            if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
                msg.data[y * msg.info.width + x] = 100;
            } else if (cost == nav2_costmap_2d::FREE_SPACE) {
                msg.data[y * msg.info.width + x] = 0;
            } else {
                msg.data[y * msg.info.width + x] = -1;
            }
        }
    }
}


CostmapPublisher::CostmapPublisher()
: rclcpp_lifecycle::LifecycleNode("semantic_costmap_publisher")
{
    RCLCPP_INFO(get_logger(), "CostmapPublisher node created");
}

nav2_util::CallbackReturn CostmapPublisher::on_configure(const rclcpp_lifecycle::State &)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "semantic_costmap", "semantic_frontier_exploration", "semantic_costmap");

    costmap_ros_->on_configure(rclcpp_lifecycle::State{});

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "costmap", rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CostmapPublisher::publishCostmap, this));

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapPublisher::on_activate(const rclcpp_lifecycle::State &)
{
    costmap_ros_->on_activate(rclcpp_lifecycle::State{});
    costmap_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CostmapPublisher::on_deactivate(const rclcpp_lifecycle::State &)
{
    costmap_ros_->on_deactivate(rclcpp_lifecycle::State{});
    costmap_pub_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

void CostmapPublisher::publishCostmap()
{
    auto costmap = costmap_ros_->getCostmap();
    if (!costmap) {
        RCLCPP_WARN(get_logger(), "Costmap not ready.");
        return;
    }

    nav_msgs::msg::OccupancyGrid msg;
    convertCostmapToOccupancyGrid(
        costmap,
        msg,
        costmap_ros_->getGlobalFrameID(),
        now());

    costmap_pub_->publish(std::move(msg));
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapPublisher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
