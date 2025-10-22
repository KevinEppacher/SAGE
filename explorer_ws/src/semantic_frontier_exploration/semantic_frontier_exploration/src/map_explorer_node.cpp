#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cmath>
#include <vector>
#include <mutex>

class LocalRaycastVisibilityNode : public rclcpp::Node
{
public:
  LocalRaycastVisibilityNode() : Node("local_raycast_visibility_node")
  {
    // --- Parameters ---
    declare_parameter("map_topic", "/inflated_map");
    declare_parameter("exploration_topic", "/exploration_visibility_map");
    declare_parameter("base_link_frame", "base_link");
    declare_parameter("update_rate_hz", 2.0);
    declare_parameter("max_range_m", 15.0);

    map_topic_ = get_parameter("map_topic").as_string();
    exploration_topic_ = get_parameter("exploration_topic").as_string();
    base_link_frame_ = get_parameter("base_link_frame").as_string();
    update_rate_ = get_parameter("update_rate_hz").as_double();
    max_range_m_ = get_parameter("max_range_m").as_double();

    // --- TF listener ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // --- ROS interfaces ---
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local(),
        std::bind(&LocalRaycastVisibilityNode::onMap, this, std::placeholders::_1));

    vis_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        exploration_topic_, rclcpp::QoS(1).transient_local());

    clear_srv_ = create_service<std_srvs::srv::Empty>(
        "/clear_exploration_map",
        std::bind(&LocalRaycastVisibilityNode::onClear, this,
                  std::placeholders::_1, std::placeholders::_2));

    // --- Timer ---
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate_),
        std::bind(&LocalRaycastVisibilityNode::updateVisibility, this));

    RCLCPP_INFO(get_logger(), "LocalRaycastVisibilityNode started (map=%s)", map_topic_.c_str());
  }

private:
  // --- ROS handles ---
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr vis_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex mutex_;

  // --- State ---
  nav_msgs::msg::OccupancyGrid static_map_;
  nav_msgs::msg::OccupancyGrid vis_map_;
  bool has_map_ = false;
  bool cleared_ = false;

  // --- Params ---
  std::string map_topic_, exploration_topic_, base_link_frame_;
  double update_rate_, max_range_m_;

  // ---------- Utility functions ----------
  static inline bool worldToMap(double wx, double wy, const nav_msgs::msg::MapMetaData &info, int &mx, int &my)
  {
    const double dx = wx - info.origin.position.x;
    const double dy = wy - info.origin.position.y;
    if (dx < 0.0 || dy < 0.0)
      return false;
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
      if (!visit(x, y))
        break;
      if (x == x1 && y == y1)
        break;
      int e2 = 2 * err;
      if (e2 >= dy)
      {
        err += dy;
        x += sx;
      }
      if (e2 <= dx)
      {
        err += dx;
        y += sy;
      }
    }
  }

  // ---------- Map subscription ----------
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    static_map_ = *msg;
    vis_map_ = *msg;
    std::fill(vis_map_.data.begin(), vis_map_.data.end(), -1); // all unknown
    has_map_ = true;
    cleared_ = false;
    RCLCPP_INFO(get_logger(), "Static map received: %u x %u (res=%.3f)",
                msg->info.width, msg->info.height, msg->info.resolution);
  }

  // ---------- Clear exploration map ----------
  void onClear(const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_map_)
      return;
    std::fill(vis_map_.data.begin(), vis_map_.data.end(), -1);
    cleared_ = true;
    RCLCPP_INFO(get_logger(), "Exploration visibility map cleared.");
    vis_map_.header.stamp = now();
    vis_pub_->publish(vis_map_);
  }

  // ---------- Timer update ----------
  void updateVisibility()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_map_ || cleared_)
      return;

    geometry_msgs::msg::TransformStamped tf_map_base;
    try
    {
      tf_map_base = tf_buffer_->lookupTransform(static_map_.header.frame_id, base_link_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    const auto &info = static_map_.info;
    const double res = info.resolution;
    const int w = info.width;
    const int h = info.height;
    const double max_cells = max_range_m_ / res;

    int x0, y0;
    if (!worldToMap(tf_map_base.transform.translation.x,
                    tf_map_base.transform.translation.y,
                    info, x0, y0))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot pose outside map bounds.");
      return;
    }

    const auto &src = static_map_.data;

    // Raycast toward all occupied cells within max range
    for (int y = std::max(0, y0 - static_cast<int>(max_cells));
         y < std::min(h, y0 + static_cast<int>(max_cells)); ++y)
    {
      for (int x = std::max(0, x0 - static_cast<int>(max_cells));
           x < std::min(w, x0 + static_cast<int>(max_cells)); ++x)
      {
        const int idx = y * w + x;
        if (src[idx] != 100)
          continue;

        // Raycast: stop at first occupied cell in static map
        bresenham(x0, y0, x, y, [&](int cx, int cy) {
          if (cx < 0 || cy < 0 || cx >= w || cy >= h)
            return false;
          int cidx = cy * w + cx;
          if (src[cidx] == 100)
          {
            vis_map_.data[cidx] = 100; // endpoint obstacle visible
            return false;              // stop ray here
          }
          if (vis_map_.data[cidx] != 100)
            vis_map_.data[cidx] = 0; // free line of sight
          return true;
        });
      }
    }

    vis_map_.header.stamp = now();
    vis_pub_->publish(vis_map_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalRaycastVisibilityNode>());
  rclcpp::shutdown();
  return 0;
}
