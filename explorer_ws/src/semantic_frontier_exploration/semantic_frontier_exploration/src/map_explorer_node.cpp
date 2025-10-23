// masked_persistent_mapper.cpp
//
// Build a persistent exploration map using a local raycast mask
// gated by the parent /inflated_map. Only cells consistent with the parent
// map (0 free or 100 occupied) are stored persistently.
//
// Clean refactor: camelCase, modular classes, fixed 4-space indentation.
//
// Parameters:
//   mapTopic              (string) default "/inflated_map"
//   explorationTopic      (string) default "/exploration_persistent_map"
//   baseLinkFrame         (string) default "base_link"
//   clearServiceName      (string) default "/clear_exploration_map"
//   updateRateHz          (double) default 2.0
//   maxRangeM             (double) default 15.0
//   stride                (int)    default 1
//
// Author: Kevin Eppacher (refined persistent mapper)

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
#include <memory>

// ========================== Raycaster ==========================
class Raycaster {
public:
    template<typename F>
    static void bresenham(int x0, int y0, int x1, int y1, F&& visit) {
        int dx = std::abs(x1 - x0);
        int dy = -std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;
        int x = x0, y = y0;

        while (true) {
            if (!visit(x, y)) break;
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
    }

    static bool worldToMap(double wx, double wy, const nav_msgs::msg::MapMetaData& info,
                           int& mx, int& my) {
        double dx = wx - info.origin.position.x;
        double dy = wy - info.origin.position.y;
        if (dx < 0.0 || dy < 0.0) return false;
        mx = static_cast<int>(dx / info.resolution);
        my = static_cast<int>(dy / info.resolution);
        return (mx >= 0 && my >= 0 &&
                mx < static_cast<int>(info.width) &&
                my < static_cast<int>(info.height));
    }
};

// ========================== PersistentMap ==========================
class PersistentMap {
public:
    void initialize(const nav_msgs::msg::OccupancyGrid& templateMap) {
        map = templateMap;
        std::fill(map.data.begin(), map.data.end(), -1);
        initialized = true;
    }

    bool isInitialized() const { return initialized; }

    void clear() {
        if (!initialized) return;
        std::fill(map.data.begin(), map.data.end(), -1);
    }

    void updateWithMask(const nav_msgs::msg::OccupancyGrid& parentMap,
                        int x0, int y0, double maxCells, int stride) {
        if (!initialized) return;

        const int width = parentMap.info.width;
        const int height = parentMap.info.height;
        const auto& src = parentMap.data;

        int rangeCells = static_cast<int>(std::round(maxCells));
        int yMin = std::max(0, y0 - rangeCells);
        int yMax = std::min(height, y0 + rangeCells);
        int xMin = std::max(0, x0 - rangeCells);
        int xMax = std::min(width, x0 + rangeCells);

        for (int y = yMin; y < yMax; ++y) {
            if (y % stride != 0) continue;
            for (int x = xMin; x < xMax; ++x) {
                if (x % stride != 0) continue;

                int idx = y * width + x;
                if (src[idx] != 100) continue;

                int dx = x - x0;
                int dy = y - y0;
                if (dx * dx + dy * dy > rangeCells * rangeCells) continue;

                Raycaster::bresenham(x0, y0, x, y, [&](int cx, int cy) {
                    if (cx < 0 || cy < 0 || cx >= width || cy >= height) return false;
                    int cidx = cy * width + cx;

                    // Stop at first occupied cell in parent map
                    if (src[cidx] == 100) {
                        map.data[cidx] = 100;
                        return false;
                    }

                    // Only mark free if parent says free
                    if (src[cidx] == 0 && map.data[cidx] != 100)
                        map.data[cidx] = 0;

                    // Unknown in parent -> stop
                    if (src[cidx] == -1)
                        return false;

                    return true;
                });
            }
        }
    }

    nav_msgs::msg::OccupancyGrid& get() { return map; }

private:
    nav_msgs::msg::OccupancyGrid map;
    bool initialized = false;
};

// ========================== ExplorerMap ==========================
class ExplorerMap : public rclcpp::Node {
public:
    ExplorerMap() : Node("explorer_map") {
        // Parameters
        mapTopic          = declare_parameter("map_topic", std::string("/inflated_map"));
        explorationTopic  = declare_parameter("exploration_topic", std::string("/exploration_persistent_map"));
        baseLinkFrame     = declare_parameter("base_link_frame", std::string("base_link"));
        clearServiceName  = declare_parameter("clear_service_name", std::string("/clear_exploration_map"));
        updateRateHz      = declare_parameter("update_rate_hz", 2.0);
        maxRangeM         = declare_parameter("max_range_m", 15.0);
        stride            = declare_parameter("stride", 1);

        // TF
        tfBuffer  = std::make_unique<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

        // Subscriptions and pubs
        mapSub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            mapTopic, rclcpp::QoS(1).transient_local(),
            std::bind(&ExplorerMap::onMap, this, std::placeholders::_1));

        mapPub = create_publisher<nav_msgs::msg::OccupancyGrid>(
            explorationTopic, rclcpp::QoS(1).transient_local());

        clearSrv = create_service<std_srvs::srv::Empty>(
            clearServiceName,
            std::bind(&ExplorerMap::onClear, this, std::placeholders::_1, std::placeholders::_2));

        timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / updateRateHz),
            std::bind(&ExplorerMap::update, this));

        RCLCPP_INFO(get_logger(), "ExplorerMap running. Parent: %s, Output: %s, Clear service: %s",
                    mapTopic.c_str(), explorationTopic.c_str(), clearServiceName.c_str());
    }

private:
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearSrv;
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::mutex mtx;

    // State
    nav_msgs::msg::OccupancyGrid parentMap;
    PersistentMap persistentMap;
    bool haveParent = false;

    // Params
    std::string mapTopic, explorationTopic, baseLinkFrame, clearServiceName;
    double updateRateHz, maxRangeM;
    int stride;

    // ------------------- Callbacks -------------------
    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx);
        parentMap = *msg;

        if (!persistentMap.isInitialized()) {
            persistentMap.initialize(*msg);
            haveParent = true;
            RCLCPP_INFO(get_logger(), "Parent map received (%u x %u, res=%.3f)",
                        msg->info.width, msg->info.height, msg->info.resolution);
        }
    }

    void onClear(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                 std::shared_ptr<std_srvs::srv::Empty::Response>) {
        std::lock_guard<std::mutex> lock(mtx);
        persistentMap.clear();
        RCLCPP_INFO(get_logger(), "Persistent exploration map cleared.");
        auto& map = persistentMap.get();
        map.header.stamp = now();
        mapPub->publish(map);
    }

    void update() {
        std::lock_guard<std::mutex> lock(mtx);
        if (!haveParent) return;

        geometry_msgs::msg::TransformStamped tfMapBase;
        try {
            tfMapBase = tfBuffer->lookupTransform(parentMap.header.frame_id, baseLinkFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
            return;
        }

        int x0, y0;
        if (!Raycaster::worldToMap(tfMapBase.transform.translation.x,
                                   tfMapBase.transform.translation.y,
                                   parentMap.info, x0, y0)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot pose outside map bounds.");
            return;
        }

        double maxCells = maxRangeM / parentMap.info.resolution;
        persistentMap.updateWithMask(parentMap, x0, y0, maxCells, stride);

        auto& out = persistentMap.get();
        out.header.stamp = now();
        mapPub->publish(out);
    }
};

// ------------------- main -------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorerMap>());
    rclcpp::shutdown();
    return 0;
}
