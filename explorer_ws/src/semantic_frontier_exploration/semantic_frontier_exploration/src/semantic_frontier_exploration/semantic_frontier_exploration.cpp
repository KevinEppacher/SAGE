#include "semantic_frontier_exploration.hpp"

struct ValueMapInfo
{
    bool observed = false;
    double score = 0.0;
};


SemanticFrontier::SemanticFrontier() : Node("cluster_node") 
{
    // Constructor implementation
    RCLCPP_INFO(this->get_logger(), "Semantic Frontier Exploration Node has been initialized.");

    getParameters();

    paramCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&SemanticFrontier::onParameterChange, this, std::placeholders::_1)
    );

    // Define subscribers
    occupancygridSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(occupancyTopic, 10,std::bind(&SemanticFrontier::occupancyGridCallback, this, std::placeholders::_1));

    valueMapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(valueMapTopic, 10,std::bind(&SemanticFrontier::valueMapCallback, this, std::placeholders::_1));

    // Define timers
    auto interval = std::chrono::duration<double>(1.0 / publishFrequency);
    RCLCPP_INFO(this->get_logger(), "Publishing with a time interval of %.3f seconds", interval.count());
    timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(interval),
        std::bind(&SemanticFrontier::timerCallback, this)
    );

    frontiers = std::make_shared<FrontierCollection>(this);

    graphNodes = std::make_shared<GraphNodeCollection>(this);

}

SemanticFrontier::~SemanticFrontier()
{
    // Destructor implementation
    RCLCPP_INFO(this->get_logger(), "Semantic Frontier Exploration Node is being destroyed.");
}

void SemanticFrontier::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    occGrid = msg;
}

void SemanticFrontier::valueMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    valueMapPcl.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *valueMapPcl);
}

void SemanticFrontier::getParameters()
{
    declare_parameter("publish_frequency", 1.0);
    declare_parameter("occupancy_topic", "/map");
    declare_parameter("value_map_topic", "/value_map_raw");

    publishFrequency = get_parameter("publish_frequency").as_double();
    occupancyTopic = get_parameter("occupancy_topic").as_string();
    valueMapTopic = get_parameter("value_map_topic").as_string();

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

rcl_interfaces::msg::SetParametersResult SemanticFrontier::onParameterChange(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated";

    for (const auto &param : params)
    {
        if (param.get_name() == "publish_frequency") {
            publishFrequency = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated publish_frequency to %.2f", publishFrequency);
        } 
    }

    return result;
}

void SemanticFrontier::timerCallback()
{
    if (!occGrid)
    {
        RCLCPP_WARN(this->get_logger(), "Occupancy grid is not available yet.");
        return;
    }

    // 1. Frontier detection
    nav_msgs::msg::OccupancyGrid frontierGrid = frontiers->detectFrontierGrid(occGrid);
    auto frontierGridPtr = std::make_shared<nav_msgs::msg::OccupancyGrid>(frontierGrid);
    std::vector<Frontier> frontiersFound = frontiers->clusterFrontierGrid(frontierGridPtr);

    graphNodes->clear();

    // 2. Build GraphNodes
    for (auto& frontier : frontiersFound)
    {
        geometry_msgs::msg::Point centroid = frontier.getCentroid();
        ValueMapInfo info = getScoreFromValueMap(valueMapPcl, centroid);
        GraphNode node;
        int id = frontier.getId();
        node.setId(id);
        node.setPosition(centroid);
        node.setScore(info.score);
        node.setIsObserved(info.observed);

        graphNodes->addNode(node);
    }

    // 3. Publish
    graphNodes->publishPosMarkers();
    graphNodes->publishGraphNodeArray();
}


ValueMapInfo SemanticFrontier::getScoreFromValueMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const geometry_msgs::msg::Point& pos)
{
    ValueMapInfo info;

    if (!cloud || cloud->empty()) 
        return info;

    constexpr float radius = 0.3f;
    const float r2 = radius * radius;

    double best = 0.0;
    bool found = false;

    for (const auto& pt : cloud->points)
    {
        float dx = pt.x - pos.x;
        float dy = pt.y - pos.y;
        float dz = pt.z - pos.z;

        if ((dx*dx + dy*dy + dz*dz) < r2)
        {
            best = std::max(best, static_cast<double>(pt.intensity));
            found = true;
        }
    }

    if (found)
    {
        info.observed = true;
        info.score = best;
    }

    return info;
}
