// pose_buffered_persistent_mapper.cpp
//
// Accumulates a persistent global occupancy grid by raycasting from all
// saved robot poses whenever /inflated_map updates.
//
// Topics:
//   sub: /inflated_map (nav_msgs/OccupancyGrid)
//   pub: /exploration_persistent_map (nav_msgs/OccupancyGrid)
//   tf:  map -> base_link
//
// Service:
//   /clear_exploration_map (std_srvs/Empty)
//
// Parameters:
//   map_topic:         "/inflated_map"
//   exploration_topic: "/exploration_persistent_map"
//   base_link_frame:   "base_link"
//   max_range_m:       15.0
//   stride:            1
//   pose_dist_thresh:  0.3   [m]
//   pose_yaw_thresh:   0.17  [rad]
//   max_poses:         50
//

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.h>

#include <deque>
#include <mutex>
#include <cmath>
#include <algorithm>

class PoseBufferedPersistentMapper : public rclcpp::Node
{
public:
  PoseBufferedPersistentMapper() : Node("pose_buffered_persistent_mapper")
  {
    declare_parameter<std::string>("map_topic", "/inflated_map");
    declare_parameter<std::string>("exploration_topic", "/exploration_persistent_map");
    declare_parameter<std::string>("base_link_frame", "base_link");
    declare_parameter<double>("max_range_m", 15.0);
    declare_parameter<int>("stride", 1);
    declare_parameter<double>("pose_dist_thresh", 0.3);
    declare_parameter<double>("pose_yaw_thresh", 0.17);
    declare_parameter<int>("max_poses", 50);

    map_topic_   = get_parameter("map_topic").as_string();
    out_topic_   = get_parameter("exploration_topic").as_string();
    base_link_   = get_parameter("base_link_frame").as_string();
    max_range_m_ = get_parameter("max_range_m").as_double();
    stride_      = std::max(1, static_cast<int>(get_parameter("stride").as_int()));
    pose_dist_thresh_ = get_parameter("pose_dist_thresh").as_double();
    pose_yaw_thresh_  = get_parameter("pose_yaw_thresh").as_double();
    max_poses_        = get_parameter("max_poses").as_int();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&PoseBufferedPersistentMapper::onMap, this, std::placeholders::_1));

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      out_topic_, rclcpp::QoS(1).transient_local());

    clear_srv_ = create_service<std_srvs::srv::Empty>(
      "/clear_exploration_map",
      std::bind(&PoseBufferedPersistentMapper::onClear, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "PoseBufferedPersistentMapper ready (parent=%s, out=%s)",
                map_topic_.c_str(), out_topic_.c_str());
  }

private:
  struct PoseRec { double x, y, yaw; };
  std::deque<PoseRec> poses_;
  size_t max_poses_{50};
  double pose_dist_thresh_{0.3};
  double pose_yaw_thresh_{0.17};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex mtx_;

  nav_msgs::msg::OccupancyGrid parent_map_;
  nav_msgs::msg::OccupancyGrid persistent_map_;
  bool have_parent_{false};

  std::string map_topic_, out_topic_, base_link_;
  double max_range_m_{15.0};
  int stride_{1};

  // --- Helpers ---
  static bool worldToMap(double wx, double wy, const nav_msgs::msg::MapMetaData &info, int &mx, int &my)
  {
    const double dx = wx - info.origin.position.x;
    const double dy = wy - info.origin.position.y;
    if (dx < 0.0 || dy < 0.0) return false;
    mx = static_cast<int>(dx / info.resolution);
    my = static_cast<int>(dy / info.resolution);
    if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
      return false;
    return true;
  }

  template <typename F>
  static void bresenham(int x0, int y0, int x1, int y1, F &&visit)
  {
    int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (true)
    {
      if (!visit(x, y)) break;
      if (x == x1 && y == y1) break;
      int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  // --- Clear service ---
  void onClear(const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!have_parent_) return;
    std::fill(persistent_map_.data.begin(), persistent_map_.data.end(), -1);
    poses_.clear();
    RCLCPP_INFO(get_logger(), "Persistent exploration map cleared.");
    persistent_map_.header.stamp = now();
    pub_->publish(persistent_map_);
  }

  // --- On new parent map ---
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    parent_map_ = *msg;

    // (Re)init persistent map if needed
    if (!have_parent_ ||
        persistent_map_.info.width != msg->info.width ||
        persistent_map_.info.height != msg->info.height ||
        persistent_map_.info.resolution != msg->info.resolution ||
        persistent_map_.header.frame_id != msg->header.frame_id)
    {
      persistent_map_ = parent_map_;
      std::fill(persistent_map_.data.begin(), persistent_map_.data.end(), -1);
      have_parent_ = true;
      RCLCPP_INFO(get_logger(), "Parent map initialized (%ux%u)", msg->info.width, msg->info.height);
    }

    // update robot pose buffer
    geometry_msgs::msg::TransformStamped tf_map_base;
    try {
      tf_map_base = tf_buffer_->lookupTransform(msg->header.frame_id, base_link_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    double x = tf_map_base.transform.translation.x;
    double y = tf_map_base.transform.translation.y;
    double yaw = tf2::getYaw(tf_map_base.transform.rotation);

    if (poses_.empty() ||
        std::hypot(x - poses_.back().x, y - poses_.back().y) > pose_dist_thresh_ ||
        std::fabs(yaw - poses_.back().yaw) > pose_yaw_thresh_)
    {
      poses_.push_back({x, y, yaw});
      if (poses_.size() > max_poses_) poses_.pop_front();
    }

    // re-integrate rays from all saved poses
    integrateAllPoses();
  }

  // --- integrate rays from all saved poses into persistent map ---
  void integrateAllPoses()
  {
    if (!have_parent_ || poses_.empty()) return;

    const auto &info = parent_map_.info;
    const auto &src  = parent_map_.data;
    const int w = info.width, h = info.height;
    const double res = info.resolution;
    const int range_cells = static_cast<int>(max_range_m_ / res);

    for (const auto &pose : poses_)
    {
      int x0, y0;
      if (!worldToMap(pose.x, pose.y, info, x0, y0)) continue;

      for (int y = std::max(0, y0 - range_cells); y < std::min(h, y0 + range_cells); ++y)
      {
        if ((y % stride_) != 0) continue;
        for (int x = std::max(0, x0 - range_cells); x < std::min(w, x0 + range_cells); ++x)
        {
          if ((x % stride_) != 0) continue;

          const int idx = y * w + x;
          if (src[idx] != 100) continue; // only raycast toward occupied
          const int dx = x - x0, dy = y - y0;
          if (dx*dx + dy*dy > range_cells * range_cells) continue;

          bresenham(x0, y0, x, y, [&](int cx, int cy) {
            if (cx < 0 || cy < 0 || cx >= w || cy >= h) return false;
            const int cidx = cy * w + cx;

            if (src[cidx] == 100) {
              persistent_map_.data[cidx] = 100;
              return false;
            }
            if (src[cidx] == 0 && persistent_map_.data[cidx] != 100) {
              persistent_map_.data[cidx] = 0;
              return true;
            }
            return false;
          });
        }
      }
    }

    persistent_map_.header.stamp = now();
    pub_->publish(persistent_map_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseBufferedPersistentMapper>());
  rclcpp::shutdown();
  return 0;
}
