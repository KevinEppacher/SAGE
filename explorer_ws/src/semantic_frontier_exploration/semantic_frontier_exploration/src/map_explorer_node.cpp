// pose_buffered_persistent_mapper.cpp
//
// Builds a persistent global occupancy grid by raycasting from all
// saved robot poses whenever /inflated_map updates.
//
// Topics:
//   sub: /inflated_map (nav_msgs/OccupancyGrid)
//   pub: /exploration_persistent_map (nav_msgs/OccupancyGrid)
//   pub: /pose_buffer_debug (geometry_msgs/PoseArray) [if debug_mode=true]
//   tf:  map -> base_link
//
// Service:
//   /clear_exploration_map (std_srvs/Empty) or custom clear_service_name
//
// Parameters (snake_case for ROS):
//   map_topic:             "/inflated_map"
//   exploration_topic:     "/exploration_persistent_map"
//   base_link_frame:       "base_link"
//   clear_service_name:    "/clear_exploration_map"
//   debug_mode:            false
//   max_range_m:           15.0
//   stride:                1
//   pose_dist_thresh:      0.3
//   pose_yaw_thresh:       0.17
//   max_poses:             50
//

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <std_srvs/srv/empty.hpp>

#include <deque>
#include <mutex>
#include <cmath>
#include <algorithm>

class PoseBufferedPersistentMapper : public rclcpp::Node
{
public:
    PoseBufferedPersistentMapper() : Node("pose_buffered_persistent_mapper")
    {
        // --- Parameters ---
        declare_parameter<std::string>("map_topic", "/inflated_map");
        declare_parameter<std::string>("exploration_topic", "/exploration_persistent_map");
        declare_parameter<std::string>("base_link_frame", "base_link");
        declare_parameter<std::string>("clear_service_name", "/clear_exploration_map");
        declare_parameter<bool>("debug_mode", false);
        declare_parameter<double>("max_range_m", 15.0);
        declare_parameter<int>("stride", 1);
        declare_parameter<double>("pose_dist_thresh", 0.3);
        declare_parameter<double>("pose_yaw_thresh", 0.17);
        declare_parameter<int>("max_poses", 50);

        mapTopic = get_parameter("map_topic").as_string();
        explorationTopic = get_parameter("exploration_topic").as_string();
        baseLinkFrame = get_parameter("base_link_frame").as_string();
        clearServiceName = get_parameter("clear_service_name").as_string();
        debugMode = get_parameter("debug_mode").as_bool();
        maxRangeM = get_parameter("max_range_m").as_double();
        stride = std::max(1, static_cast<int>(get_parameter("stride").as_int()));
        poseDistThresh = get_parameter("pose_dist_thresh").as_double();
        poseYawThresh = get_parameter("pose_yaw_thresh").as_double();
        maxPoses = get_parameter("max_poses").as_int();

        tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

        mapSub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            mapTopic, rclcpp::QoS(1).transient_local(),
            std::bind(&PoseBufferedPersistentMapper::onMapUpdate, this, std::placeholders::_1));

        mapPub = create_publisher<nav_msgs::msg::OccupancyGrid>(
            explorationTopic, rclcpp::QoS(1).transient_local());

        if (debugMode) {
            posePub = create_publisher<geometry_msgs::msg::PoseArray>(
                "pose_buffer_debug", rclcpp::QoS(1).transient_local());
        }

        clearSrv = create_service<std_srvs::srv::Empty>(
            clearServiceName,
            std::bind(&PoseBufferedPersistentMapper::onClearRequest, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "PoseBufferedPersistentMapper ready (parent=%s → out=%s, clear=%s, debug=%s)",
                    mapTopic.c_str(), explorationTopic.c_str(),
                    clearServiceName.c_str(), debugMode ? "true" : "false");
    }

private:
    struct PoseRec { double x, y, yaw; };
    std::deque<PoseRec> poseBuffer;
    size_t maxPoses{50};
    double poseDistThresh{0.3};
    double poseYawThresh{0.17};

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearSrv;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::mutex mtx;

    nav_msgs::msg::OccupancyGrid parentMap;
    nav_msgs::msg::OccupancyGrid persistentMap;
    bool haveParent{false};
    bool debugMode{false};

    std::string mapTopic, explorationTopic, baseLinkFrame, clearServiceName;
    double maxRangeM{15.0};
    int stride{1};

    // --- Utility functions ---
    static bool worldToMap(double wx, double wy,
                           const nav_msgs::msg::MapMetaData &info,
                           int &mx, int &my)
    {
        double dx = wx - info.origin.position.x;
        double dy = wy - info.origin.position.y;
        if (dx < 0.0 || dy < 0.0) return false;
        mx = static_cast<int>(dx / info.resolution);
        my = static_cast<int>(dy / info.resolution);
        if (mx < 0 || my < 0 ||
            mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
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

    // --- Service callback ---
    void onClearRequest(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!haveParent) return;
        std::fill(persistentMap.data.begin(), persistentMap.data.end(), -1);
        poseBuffer.clear();
        RCLCPP_INFO(get_logger(), "Persistent map and pose buffer cleared.");
        persistentMap.header.stamp = now();
        mapPub->publish(persistentMap);
        if (debugMode) publishPoseArray();
    }

    // --- Main callback on map update ---
    void onMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        parentMap = *msg;

        if (!haveParent ||
            persistentMap.info.width != msg->info.width ||
            persistentMap.info.height != msg->info.height ||
            persistentMap.info.resolution != msg->info.resolution ||
            persistentMap.header.frame_id != msg->header.frame_id)
        {
            persistentMap = parentMap;
            std::fill(persistentMap.data.begin(), persistentMap.data.end(), -1);
            haveParent = true;
            RCLCPP_INFO(get_logger(), "Parent map initialized (%ux%u)",
                        msg->info.width, msg->info.height);
        }

        geometry_msgs::msg::TransformStamped tfMapBase;
        try {
            tfMapBase = tfBuffer->lookupTransform(
                msg->header.frame_id, baseLinkFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF lookup failed: %s", ex.what());
            return;
        }

        double x = tfMapBase.transform.translation.x;
        double y = tfMapBase.transform.translation.y;
        double yaw = tf2::getYaw(tfMapBase.transform.rotation);

        if (poseBuffer.empty() ||
            std::hypot(x - poseBuffer.back().x, y - poseBuffer.back().y) > poseDistThresh ||
            std::fabs(yaw - poseBuffer.back().yaw) > poseYawThresh)
        {
            poseBuffer.push_back({x, y, yaw});
            if (poseBuffer.size() > maxPoses) poseBuffer.pop_front();
        }

        integrateAllPoses();
        if (debugMode) publishPoseArray();
    }

    // --- Publish PoseArray for debugging ---
    void publishPoseArray()
    {
        if (!posePub || poseBuffer.empty()) return;

        geometry_msgs::msg::PoseArray array;
        array.header.frame_id = parentMap.header.frame_id;
        array.header.stamp = now();

        for (const auto &p : poseBuffer) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p.yaw);
            pose.orientation = tf2::toMsg(q);
            array.poses.push_back(pose);
        }

        posePub->publish(array);
    }

    // --- Core integration logic ---
    void integrateAllPoses()
    {
        if (!haveParent || poseBuffer.empty()) return;

        const auto &info = parentMap.info;
        const auto &src = parentMap.data;
        const int w = info.width, h = info.height;
        const double res = info.resolution;
        const int rangeCells = static_cast<int>(maxRangeM / res);

        for (const auto &pose : poseBuffer)
        {
            int x0, y0;
            if (!worldToMap(pose.x, pose.y, info, x0, y0)) continue;

            for (int y = std::max(0, y0 - rangeCells); y < std::min(h, y0 + rangeCells); ++y)
            {
                if ((y % stride) != 0) continue;
                for (int x = std::max(0, x0 - rangeCells); x < std::min(w, x0 + rangeCells); ++x)
                {
                    if ((x % stride) != 0) continue;
                    const int idx = y * w + x;
                    if (src[idx] != 100) continue;
                    const int dx = x - x0, dy = y - y0;
                    if (dx * dx + dy * dy > rangeCells * rangeCells) continue;

                    bresenham(x0, y0, x, y, [&](int cx, int cy) {
                        if (cx < 0 || cy < 0 || cx >= w || cy >= h) return false;
                        const int cidx = cy * w + cx;

                        if (src[cidx] == 100) {
                            persistentMap.data[cidx] = 100;
                            return false;
                        }
                        if (src[cidx] == 0 && persistentMap.data[cidx] != 100) {
                            persistentMap.data[cidx] = 0;
                            return true;
                        }
                        return false;
                    });
                }
            }
        }

        persistentMap.header.stamp = now();
        mapPub->publish(persistentMap);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseBufferedPersistentMapper>());
    rclcpp::shutdown();
    return 0;
}
