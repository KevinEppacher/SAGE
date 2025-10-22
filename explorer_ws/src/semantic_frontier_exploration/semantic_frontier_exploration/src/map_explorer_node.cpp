// masked_persistent_mapper.cpp
//
// Build a persistent global occupancy map by *masking* local raycast results
// with the parent map (/inflated_map). We only commit cells to the persistent
// map when the local raycast mask (0 free, 100 occupied) equals the parent map value.
// This gives robust, drift-resistant accumulation.
//
// Topics:
//   sub: /inflated_map (nav_msgs/OccupancyGrid)
//   pub: /exploration_persistent_map (nav_msgs/OccupancyGrid)
// Service:
//   /clear_exploration_map  (std_srvs/Empty)
//
// Parameters:
//   map_topic:               "/inflated_map"
//   exploration_topic:       "/exploration_persistent_map"
//   base_link_frame:         "base_link"
//   update_rate_hz:          2.0
//   max_range_m:             15.0
//   stride:                  1   (optional subsampling of parent cells within radius)
//
// Notes:
//   - Raycasting stops at first occupied cell in parent map.
//   - TF lookup uses latest (TimePointZero) to avoid extrapolation issues.
//   - No inflated output here (another node can inflate if needed).

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/empty.hpp>

#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>

class MaskedPersistentMapper : public rclcpp::Node
{
public:
  MaskedPersistentMapper() : Node("masked_persistent_mapper")
  {
    // --- Parameters ---
    declare_parameter<std::string>("map_topic", "/inflated_map");
    declare_parameter<std::string>("exploration_topic", "/exploration_persistent_map");
    declare_parameter<std::string>("base_link_frame", "base_link");
    declare_parameter<double>("update_rate_hz", 2.0);
    declare_parameter<double>("max_range_m", 15.0);
    declare_parameter<int>("stride", 1);

    map_topic_         = get_parameter("map_topic").as_string();
    exploration_topic_ = get_parameter("exploration_topic").as_string();
    base_link_frame_   = get_parameter("base_link_frame").as_string();
    update_rate_       = get_parameter("update_rate_hz").as_double();
    max_range_m_       = get_parameter("max_range_m").as_double();
    stride_            = std::max(1, static_cast<int>(get_parameter("stride").as_int()));

    // --- TF ---
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // --- ROS I/O ---
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&MaskedPersistentMapper::onMap, this, std::placeholders::_1));

    persistent_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      exploration_topic_, rclcpp::QoS(1).transient_local());

    clear_srv_ = create_service<std_srvs::srv::Empty>(
      "/clear_exploration_map",
      std::bind(&MaskedPersistentMapper::onClear, this, std::placeholders::_1, std::placeholders::_2));

    // Timer tick
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_),
      std::bind(&MaskedPersistentMapper::tick, this));

    RCLCPP_INFO(get_logger(), "MaskedPersistentMapper up. parent=%s -> out=%s",
                map_topic_.c_str(), exploration_topic_.c_str());
  }

private:
  // ==== Members ====
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr persistent_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex mtx_;

  nav_msgs::msg::OccupancyGrid parent_map_;      // latest /inflated_map
  nav_msgs::msg::OccupancyGrid persistent_map_;  // global exploration memory
  bool have_parent_{false};
  bool just_cleared_{false};

  std::string map_topic_, exploration_topic_, base_link_frame_;
  double update_rate_{2.0}, max_range_m_{15.0};
  int stride_{1};

  // ==== Utils ====
  static inline bool worldToMap(double wx, double wy,
                                const nav_msgs::msg::MapMetaData &info,
                                int &mx, int &my)
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
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (true)
    {
      if (!visit(x,y)) break;
      if (x == x1 && y == y1) break;
      int e2 = 2*err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  // ==== Callbacks ====
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    parent_map_ = *msg;

    // (Re)initialize persistent map if metadata changed or first time
    if (!have_parent_ ||
        persistent_map_.info.width        != parent_map_.info.width ||
        persistent_map_.info.height       != parent_map_.info.height ||
        persistent_map_.info.resolution   != parent_map_.info.resolution ||
        persistent_map_.header.frame_id   != parent_map_.header.frame_id ||
        persistent_map_.info.origin.position.x != parent_map_.info.origin.position.x ||
        persistent_map_.info.origin.position.y != parent_map_.info.origin.position.y)
    {
      persistent_map_ = parent_map_;
      std::fill(persistent_map_.data.begin(), persistent_map_.data.end(), -1);
      have_parent_ = true;
      just_cleared_ = false;
      RCLCPP_INFO(get_logger(), "Parent map received: %ux%u, res=%.3f",
                  parent_map_.info.width, parent_map_.info.height, parent_map_.info.resolution);
    }
  }

  void onClear(const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!have_parent_) return;
    std::fill(persistent_map_.data.begin(), persistent_map_.data.end(), -1);
    just_cleared_ = true;
    RCLCPP_INFO(get_logger(), "Exploration persistent map cleared.");
    persistent_map_.header.stamp = now();
    persistent_pub_->publish(persistent_map_);
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!have_parent_) return;

    // Robot pose in map frame (latest TF to avoid time issues)
    geometry_msgs::msg::TransformStamped tf_map_base;
    try {
      tf_map_base = tf_buffer_->lookupTransform(parent_map_.header.frame_id, base_link_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    const auto &info = parent_map_.info;
    const int w = info.width, h = info.height;
    const double res = info.resolution;
    const int range_cells = static_cast<int>(std::round(max_range_m_ / res));

    int x0, y0;
    if (!worldToMap(tf_map_base.transform.translation.x,
                    tf_map_base.transform.translation.y, info, x0, y0))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot pose out of bounds.");
      return;
    }

    // --- Build a local raycast MASK (unknown=-1, free=0, occ=100) ---
    // We don't allocate a full image; we directly apply results into persistent_map_
    // but *only* when they match parent_map_.

    const auto &src = parent_map_.data;

    const int y_min = std::max(0, y0 - range_cells);
    const int y_max = std::min(h, y0 + range_cells);
    const int x_min = std::max(0, x0 - range_cells);
    const int x_max = std::min(w, x0 + range_cells);

    for (int y = y_min; y < y_max; ++y)
    {
      if ((y % stride_) != 0) continue;
      for (int x = x_min; x < x_max; ++x)
      {
        if ((x % stride_) != 0) continue;

        const int idx = y * w + x;
        if (src[idx] != 100) continue;                 // only cast towards obstacles
        // Stop distance gate (circle)
        const int dx = x - x0, dy = y - y0;
        if (dx*dx + dy*dy > range_cells * range_cells) continue;

        // Raycast: stop at first OCCUPIED in parent map
        bresenham(x0, y0, x, y, [&](int cx, int cy) {
          if (cx < 0 || cy < 0 || cx >= w || cy >= h) return false;
          const int cidx = cy * w + cx;

          // If parent says occupied at this cell → endpoint; commit 100 and stop
          if (src[cidx] == 100)
          {
            // Commit to persistent only if parent agrees (it does) and not already set
            if (persistent_map_.data[cidx] != 100)
              persistent_map_.data[cidx] = 100;
            return false;
          }

          // Otherwise along the line is free in the parent map *iff* parent is 0
          if (src[cidx] == 0)
          {
            // Only write free if not already occupied in persistent
            if (persistent_map_.data[cidx] != 100)
              persistent_map_.data[cidx] = 0;
            return true; // continue
          }

          // If parent is unknown (-1) we don't learn anything; stop early
          return false;
        });
      }
    }

    // Publish the persistent map snapshot
    persistent_map_.header.stamp = now();
    persistent_pub_->publish(persistent_map_);

    // Once we published after clear, reset flag (no behavior change; just informational)
    if (just_cleared_) just_cleared_ = false;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaskedPersistentMapper>());
  rclcpp::shutdown();
  return 0;
}
