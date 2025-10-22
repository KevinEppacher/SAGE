// map_raytrace_exploration_node.cpp
// Builds a raytraced /exploration_map from parent /map, inflates it, and supports clearing via service.
// Keeps your inflation logic, but wraps it in a small helper for modularity.

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <cmath>
#include <optional>

class MapInflator {
public:
  struct Params {
    double radius_m = 0.25;
    int iterations = 1;
    std::string kernel_type = "rect"; // rect, ellipse, cross
  };

  explicit MapInflator(const Params &p) : p_(p) {}

  void updateParams(const Params &p) { p_ = p; }

  // Apply inflation on a binary-occupied CV_8U image (1=occ, 0=free/unknown)
  cv::Mat inflate(const cv::Mat &occ_binary, double resolution) const {
    int pixels = std::max(1, static_cast<int>(std::round(p_.radius_m / resolution)));
    if (pixels % 2 == 0) pixels++;
    int shape = cv::MORPH_RECT;
    if (p_.kernel_type == "ellipse") shape = cv::MORPH_ELLIPSE;
    else if (p_.kernel_type == "cross") shape = cv::MORPH_CROSS;

    cv::Mat kernel = cv::getStructuringElement(shape, cv::Size(pixels, pixels));
    cv::Mat inflated;
    cv::dilate(occ_binary, inflated, kernel, cv::Point(-1, -1), p_.iterations);
    return inflated;
  }

private:
  Params p_;
};

class ExplorationBuilder {
public:
  // Build a raytraced exploration grid from a parent map and robot world pose.
  // - Parent map values: 100=occ, 0=free, -1=unknown
  // - Output exploration: unknown by default, set free along rays, propagate hard obstacles from parent
  static void buildRaytraced(const nav_msgs::msg::OccupancyGrid &parent,
                             double robot_x_w, double robot_y_w,
                             int beams, double max_range_m,
                             nav_msgs::msg::OccupancyGrid &exploration_out)
  {
    const auto &info = parent.info;
    const int W = static_cast<int>(info.width);
    const int H = static_cast<int>(info.height);
    const double res = info.resolution;

    exploration_out = parent; // copy metadata/header; we’ll rewrite data
    exploration_out.data.assign(W * H, -1);

    // Always keep obstacles from parent (so exploration has the same walls)
    for (int i = 0; i < W * H; ++i) {
      if (parent.data[i] == 100) {
        exploration_out.data[i] = 100;
      }
    }

    // Convert world -> map index
    auto worldToMap = [&](double x, double y, int &mx, int &my) -> bool {
      const double ox = info.origin.position.x;
      const double oy = info.origin.position.y;
      int ix = static_cast<int>(std::floor((x - ox) / res));
      int iy = static_cast<int>(std::floor((y - oy) / res));
      if (ix < 0 || iy < 0 || ix >= W || iy >= H) return false;
      mx = ix; my = iy;
      return true;
    };

    // Ray traversal using DDA on grid
    auto traverseRay = [&](double sx_w, double sy_w, double ex_w, double ey_w) {
      int sx, sy, ex, ey;
      if (!worldToMap(sx_w, sy_w, sx, sy)) return;
      if (!worldToMap(ex_w, ey_w, ex, ey)) {
        // Clamp end to map bounds by advancing until we leave, but we can just target the far cell
        // outside; DDA below will stop when out-of-bounds.
      }

      // DDA setup
      int x = sx;
      int y = sy;

      double dx = (ex_w - sx_w);
      double dy = (ey_w - sy_w);
      double len = std::hypot(dx, dy);
      if (len < 1e-9) return;

      double dirx = dx / len;
      double diry = dy / len;

      // Convert start to continuous cell coords
      // Continuous position in world, we’ll step cell-by-cell using tMax/tDelta
      // Map cell boundaries (Amanatides & Woo)
      double ox = info.origin.position.x;
      double oy = info.origin.position.y;

    //   auto cellToWorld = [&](int cx, int cy) {
    //     return std::pair<double,double>(ox + (cx + 0.5) * res, oy + (cy + 0.5) * res);
    //   };

      // Compute parametric stepping
      int stepX = (dirx > 0) ? 1 : -1;
      int stepY = (diry > 0) ? 1 : -1;

    //   auto frac = [](double v) { return v - std::floor(v); };

      // Starting world pos
      double rx = sx_w;
      double ry = sy_w;

      // Convert current world pos to cell-edge crossing distances
      auto wx_to_cell = [&](double w) { return (w - ox) / res; };
      double cellXf = wx_to_cell(rx);
      double cellYf = (ry - oy) / res;

      // Distances to next grid lines
      double nextVx = (stepX > 0) ? (std::floor(cellXf) + 1.0 - cellXf) : (cellXf - std::floor(cellXf));
      double nextVy = (stepY > 0) ? (std::floor(cellYf) + 1.0 - cellYf) : (cellYf - std::floor(cellYf));

      // tDelta in meters to cross one cell horizontally/vertically
      double tDeltaX = (std::abs(dirx) < 1e-9) ? 1e18 : (res / std::abs(dirx));
      double tDeltaY = (std::abs(diry) < 1e-9) ? 1e18 : (res / std::abs(diry));

      // tMax: distance to the first vertical/horizontal boundary
      double tMaxX = (std::abs(dirx) < 1e-9) ? 1e18 : (nextVx * res / std::abs(dirx));
      double tMaxY = (std::abs(diry) < 1e-9) ? 1e18 : (nextVy * res / std::abs(diry));

      double traveled = 0.0;

      // Mark the start cell as free (if not an obstacle)
      auto setFree = [&](int cx, int cy) {
        if (cx < 0 || cy < 0 || cx >= W || cy >= H) return;
        int idx = cy * W + cx;
        if (exploration_out.data[idx] != 100) {
          exploration_out.data[idx] = 0; // free along the line-of-sight
        }
      };

      setFree(x, y);

      // Step until we hit an obstacle, exceed max range, or exit map
      while (true) {
        if (tMaxX < tMaxY) {
          x += stepX;
          traveled += tMaxX;
          tMaxY -= tMaxX;
          tMaxX = tDeltaX;
        } else {
          y += stepY;
          traveled += tMaxY;
          tMaxX -= tMaxY;
          tMaxY = tDeltaY;
        }

        if (traveled > len) break; // reached target end point
        if (traveled > max_range_m) break;

        if (x < 0 || y < 0 || x >= W || y >= H) break;

        int idx = y * W + x;
        // Stop at parent obstacle; do not mark obstacle cell free
        if (parent.data[idx] == 100) {
          // Optionally mark hit cell as obstacle (already done in copy)
          break;
        }

        // If parent knows it's free, we can mark it free; unknown (-1) becomes seen-free along beam
        setFree(x, y);
      }
    };

    // Robot start in world; cast beams around 360°
    int r_mx, r_my;
    if (!worldToMap(robot_x_w, robot_y_w, r_mx, r_my)) {
      // Robot currently outside parent map bounds; keep exploration unknown except hard obstacles already copied
      return;
    }

    const double two_pi = 2.0 * M_PI;
    for (int b = 0; b < beams; ++b) {
      double ang = (static_cast<double>(b) / static_cast<double>(beams)) * two_pi;
      double ex = robot_x_w + std::cos(ang) * max_range_m;
      double ey = robot_y_w + std::sin(ang) * max_range_m;
      traverseRay(robot_x_w, robot_y_w, ex, ey);
    }
  }
};

class MapRaytraceExplorationNode : public rclcpp::Node {
public:
  MapRaytraceExplorationNode()
  : Node("map_raytrace_exploration_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- Parameters ----
    declare_parameter<std::string>("parent_map_topic", "/map");
    declare_parameter<std::string>("exploration_map_topic", "/exploration_map");
    declare_parameter<std::string>("input_qos", "reliable");   // reliable | best_effort
    declare_parameter<std::string>("output_qos", "reliable");  // reliable | best_effort
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("update_rate_hz", 5.0);          // TF + build loop
    declare_parameter<int>("ray_beams", 720);
    declare_parameter<double>("ray_max_range_m", 30.0);

    // Inflation parameters (kept intact, but modular)
    declare_parameter<double>("inflation_radius", 0.25);
    declare_parameter<int>("inflation_iterations", 1);
    declare_parameter<std::string>("kernel_type", "rect");

    loadParameters();
    setupQoS();

    // ---- Communication ----
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      parent_map_topic_, input_qos_,
      std::bind(&MapRaytraceExplorationNode::onParentMap, this, std::placeholders::_1));

    exploration_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      exploration_map_topic_, output_qos_);

    clear_srv_ = create_service<std_srvs::srv::Empty>(
      "clear_exploration_map",
      std::bind(&MapRaytraceExplorationNode::onClear, this,
                std::placeholders::_1, std::placeholders::_2));

    // ---- Dynamic params ----
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&MapRaytraceExplorationNode::onParamChange, this, std::placeholders::_1));

    // ---- TF worker thread ----
    run_thread_.store(true);
    worker_ = std::thread(&MapRaytraceExplorationNode::tfAndBuildLoop, this);

    RCLCPP_INFO(get_logger(),
      "Exploration node up. parent: %s | exploration: %s | beams: %d | max_range: %.1fm | rate: %.1f Hz",
      parent_map_topic_.c_str(), exploration_map_topic_.c_str(), ray_beams_, ray_max_range_m_, update_rate_hz_);
  }

  ~MapRaytraceExplorationNode() override {
    run_thread_.store(false);
    if (worker_.joinable()) worker_.join();
  }

private:
  // ===== ROS interfaces =====
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr exploration_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::QoS input_qos_{10};
  rclcpp::QoS output_qos_{1};

  // ===== TF =====
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string global_frame_{"map"};
  std::string base_frame_{"base_link"};

  // ===== Params =====
  std::string parent_map_topic_;
  std::string exploration_map_topic_;
  std::string input_qos_str_;
  std::string output_qos_str_;
  double update_rate_hz_{5.0};
  int ray_beams_{720};
  double ray_max_range_m_{30.0};

  MapInflator inflator_{MapInflator::Params{}};
  MapInflator::Params infl_params_;

  // ===== State =====
  std::mutex mtx_;
  std::optional<nav_msgs::msg::OccupancyGrid> parent_map_;
  std::optional<nav_msgs::msg::OccupancyGrid> exploration_map_;
  std::atomic<bool> need_clear_{false};
  std::atomic<bool> have_pose_{false};
  double robot_x_{0.0}, robot_y_{0.0};

  // ===== Worker thread =====
  std::atomic<bool> run_thread_{false};
  std::thread worker_;

  // ----------------- Setup helpers -----------------
  void loadParameters() {
    parent_map_topic_       = get_parameter("parent_map_topic").as_string();
    exploration_map_topic_  = get_parameter("exploration_map_topic").as_string();
    input_qos_str_          = get_parameter("input_qos").as_string();
    output_qos_str_         = get_parameter("output_qos").as_string();
    global_frame_           = get_parameter("global_frame").as_string();
    base_frame_             = get_parameter("base_frame").as_string();
    update_rate_hz_         = get_parameter("update_rate_hz").as_double();
    ray_beams_              = get_parameter("ray_beams").as_int();
    ray_max_range_m_        = get_parameter("ray_max_range_m").as_double();

    infl_params_.radius_m   = get_parameter("inflation_radius").as_double();
    infl_params_.iterations = get_parameter("inflation_iterations").as_int();
    infl_params_.kernel_type= get_parameter("kernel_type").as_string();
    inflator_.updateParams(infl_params_);
  }

  void setupQoS() {
    input_qos_ = (input_qos_str_ == "best_effort")
                  ? rclcpp::QoS(10).best_effort()
                  : rclcpp::QoS(10).reliable();

    output_qos_ = (output_qos_str_ == "best_effort")
                  ? rclcpp::QoS(1).best_effort().transient_local()
                  : rclcpp::QoS(1).reliable().transient_local();
  }

  // ----------------- Callbacks -----------------
  void onParentMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::scoped_lock lk(mtx_);
    parent_map_ = *msg;
    // Ensure exploration map meta matches parent if not yet created
    if (!exploration_map_.has_value()) {
      exploration_map_ = *msg;
      exploration_map_->data.assign(msg->info.width * msg->info.height, -1);
    }
  }

  void onClear(const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) {
    need_clear_.store(true);
    RCLCPP_INFO(get_logger(), "Requested exploration map clear.");
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    bool comm_reconf = false;
    bool infl_changed = false;

    for (const auto &p : params) {
      const auto &name = p.get_name();
      if (name == "inflation_radius") { infl_params_.radius_m = p.as_double(); infl_changed = true; }
      else if (name == "inflation_iterations") { infl_params_.iterations = p.as_int(); infl_changed = true; }
      else if (name == "kernel_type") { infl_params_.kernel_type = p.as_string(); infl_changed = true; }
      else if (name == "ray_beams") { ray_beams_ = p.as_int(); }
      else if (name == "ray_max_range_m") { ray_max_range_m_ = p.as_double(); }
      else if (name == "update_rate_hz") { update_rate_hz_ = p.as_double(); }
      else if (name == "global_frame") { global_frame_ = p.as_string(); }
      else if (name == "base_frame") { base_frame_ = p.as_string(); }
      else if (name == "parent_map_topic" || name == "exploration_map_topic" ||
               name == "input_qos" || name == "output_qos") {
        comm_reconf = true;
      }
    }

    if (infl_changed) {
      inflator_.updateParams(infl_params_);
      RCLCPP_INFO(get_logger(), "Updated inflation params: r=%.3f, it=%d, kernel=%s",
                  infl_params_.radius_m, infl_params_.iterations, infl_params_.kernel_type.c_str());
    }

    if (comm_reconf) {
      loadParameters();
      setupQoS();
      // Recreate pubs/subs
      map_sub_.reset();
      exploration_pub_.reset();

      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        parent_map_topic_, input_qos_,
        std::bind(&MapRaytraceExplorationNode::onParentMap, this, std::placeholders::_1));

      exploration_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        exploration_map_topic_, output_qos_);

      RCLCPP_INFO(get_logger(), "Reconfigured communication setup.");
    }

    return res;
  }

  // ----------------- Worker loop -----------------
  void tfAndBuildLoop() {
    rclcpp::Rate rate(update_rate_hz_ > 0 ? update_rate_hz_ : 5.0);

    while (rclcpp::ok() && run_thread_.load()) {
      // 1) Get robot pose in map
      try {
        auto tf = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
        robot_x_ = tf.transform.translation.x;
        robot_y_ = tf.transform.translation.y;
        have_pose_.store(true);
      } catch (const std::exception &e) {
        have_pose_.store(false);
      }

      // 2) If we have a parent map and a pose, (re)build exploration
      std::optional<nav_msgs::msg::OccupancyGrid> parent_copy;
      {
        std::scoped_lock lk(mtx_);
        if (parent_map_.has_value()) parent_copy = parent_map_;
      }

      if (parent_copy.has_value()) {
        // Handle clear request
        if (need_clear_.load()) {
          std::scoped_lock lk(mtx_);
          exploration_map_ = parent_copy; // copy metadata
          exploration_map_->data.assign(parent_copy->info.width * parent_copy->info.height, -1);
          need_clear_.store(false);
        }

        if (have_pose_.load()) {
        nav_msgs::msg::OccupancyGrid local_exploration;
        ExplorationBuilder::buildRaytraced(
            *parent_copy, robot_x_, robot_y_, ray_beams_, ray_max_range_m_, local_exploration);

        // Inflate obstacles
        const int W = static_cast<int>(local_exploration.info.width);
        const int H = static_cast<int>(local_exploration.info.height);
        cv::Mat occ_bin(H, W, CV_8U, cv::Scalar(0));
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
            int idx = y * W + x;
            if (local_exploration.data[idx] == 100) occ_bin.at<uint8_t>(y, x) = 1;
            }
        }
        cv::Mat inflated = inflator_.inflate(occ_bin, local_exploration.info.resolution);

        // Merge into persistent map
        std::scoped_lock lk(mtx_);
        if (!exploration_map_.has_value()) {
            exploration_map_ = local_exploration;  // initialize first time
        }

        auto &expl = exploration_map_.value();
        const int total = W * H;
        for (int i = 0; i < total; ++i) {
            // Always copy inflated obstacles from parent
            if (inflated.at<uint8_t>(i / W, i % W) > 0)
            expl.data[i] = 100;
            else if (local_exploration.data[i] == 0 && expl.data[i] == -1)
            expl.data[i] = 0; // mark newly seen free space
        }

        expl.header.stamp = now();
        exploration_pub_->publish(expl);
        }
      }

      rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapRaytraceExplorationNode>();
  // Multi-threaded executor recommended because TF runs callbacks internally
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
