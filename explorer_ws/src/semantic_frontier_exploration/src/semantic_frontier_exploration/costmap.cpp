#include "costmap.hpp"
#include <cmath>

Costmap::Costmap() : Node("costmap_publisher"), resolution_(0.05), width_(200), height_(200)
{
    origin_x_ = -width_ * resolution_ / 2.0;
    origin_y_ = -height_ * resolution_ / 2.0;

    occupancy_grid_.info.resolution = resolution_;
    occupancy_grid_.info.width = width_;
    occupancy_grid_.info.height = height_;
    occupancy_grid_.info.origin.position.x = origin_x_;
    occupancy_grid_.info.origin.position.y = origin_y_;
    occupancy_grid_.info.origin.orientation.w = 1.0;

    occupancy_grid_.data.resize(width_ * height_, -1);  // initialize as unknown

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Costmap::scanCallback, this, std::placeholders::_1));

    occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy_grid", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&Costmap::timerCallback, this));
}

Costmap::~Costmap() {}

void Costmap::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    last_scan_ = msg;
    updateOccupancyGrid();
}

void Costmap::updateOccupancyGrid()
{
    if (!last_scan_) return;

    std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1); // unknown

    for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
        float range = last_scan_->ranges[i];
        if (range < last_scan_->range_min || range > last_scan_->range_max) continue;

        float angle = last_scan_->angle_min + i * last_scan_->angle_increment;
        float x = range * std::cos(angle);
        float y = range * std::sin(angle);

        // convert world to grid coordinates
        int gx = static_cast<int>((x - origin_x_) / resolution_);
        int gy = static_cast<int>((y - origin_y_) / resolution_);

        // mirror over Y and X axis (ZY-plane)
        gx = width_ - 1 - gx;
        gy = height_ - 1 - gy;

        // draw free space along the ray
        float step_size = 0.02f;
        for (float r = 0.0f; r < range; r += step_size) {
            float fx = r * std::cos(angle);
            float fy = r * std::sin(angle);

            int free_x = width_ - 1 - static_cast<int>((fx - origin_x_) / resolution_);
            int free_y = height_ - 1 - static_cast<int>((fy - origin_y_) / resolution_);

            if (free_x >= 0 && free_x < width_ && free_y >= 0 && free_y < height_) {
                int idx = free_x + free_y * width_;
                if (occupancy_grid_.data[idx] == -1)
                    occupancy_grid_.data[idx] = 0; // free
            }
        }

        // mark obstacle cell
        if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
            int idx = gx + gy * width_;
            occupancy_grid_.data[idx] = 100; // occupied
        }
    }

    occupancy_grid_.header.stamp = now();
    occupancy_grid_.header.frame_id = "map";
}


void Costmap::timerCallback()
{
    occ_grid_pub_->publish(occupancy_grid_);
}
