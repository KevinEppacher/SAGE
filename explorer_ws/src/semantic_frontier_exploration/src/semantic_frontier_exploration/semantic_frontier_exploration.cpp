#include "semantic_frontier_exploration.hpp"

void visualizeOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid, const std::string& window_name = "Occupancy Grid")
{
    if (!grid || grid->data.empty()) return;

    int width = grid->info.width;
    int height = grid->info.height;
    cv::Mat image(height, width, CV_8UC1);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int i = x + (height - 1 - y) * width;
            int val = grid->data[i];
            uint8_t pixel = (val == 0) ? 255 : (val > 0) ? 0 : 127;
            image.at<uchar>(y, x) = pixel;
        }
    }

    cv::imshow(window_name, image);
    cv::waitKey(1);
}

SemanticFrontier::SemanticFrontier() : Node("cluster_node") 
{
    // Constructor implementation
    RCLCPP_INFO(this->get_logger(), "Semantic Frontier Exploration Node has been initialized.");

    getParameters();

    // Define subscribers
    occpancygridSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10,
      std::bind(&SemanticFrontier::occupancyGridCallback, this, std::placeholders::_1));

    // Define timers
    auto interval = std::chrono::duration<double>(1.0 / publishFrequency);
    RCLCPP_INFO(this->get_logger(), "Publishing with a time interval of %.3f seconds", interval.count());
    timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(interval),
        std::bind(&SemanticFrontier::timerCallback, this)
    );

    frontier = std::make_shared<Frontier>(this);

}

SemanticFrontier::~SemanticFrontier()
{
    // Destructor implementation
    RCLCPP_INFO(this->get_logger(), "Semantic Frontier Exploration Node is being destroyed.");
}

// Subscriber callback functions
void SemanticFrontier::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    occGrid = msg;
}

void SemanticFrontier::getParameters()
{
    this->declare_parameter("publish_frequency", 10.0);

    this->get_parameter("publish_frequency", publishFrequency);


    RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
    for (const auto &name : this->list_parameters({}, 10).names)
    {
        auto param = this->get_parameter(name);
        switch (param.get_type())
        {
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                RCLCPP_INFO(this->get_logger(), " - %s: %f", name.c_str(), param.as_double());
                break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                RCLCPP_INFO(this->get_logger(), " - %s: %ld", name.c_str(), param.as_int());
                break;
            case rclcpp::ParameterType::PARAMETER_STRING:
                RCLCPP_INFO(this->get_logger(), " - %s: %s", name.c_str(), param.as_string().c_str());
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
                RCLCPP_INFO(this->get_logger(), " - %s: %s", name.c_str(), param.as_bool() ? "true" : "false");
                break;
            default:
                RCLCPP_INFO(this->get_logger(), " - %s: [unsupported type]", name.c_str());
                break;
        }
    }
}

void SemanticFrontier::timerCallback()
{
    if(occGrid.get() == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Occupancy grid is not available yet.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Timer callback triggered. Occupancy grid size: %zu", occGrid->data.size());
    visualizeOccupancyGrid(occGrid);
}