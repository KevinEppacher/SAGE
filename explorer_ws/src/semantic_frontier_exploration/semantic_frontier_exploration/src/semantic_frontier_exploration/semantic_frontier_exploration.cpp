#include "semantic_frontier_exploration.hpp"

SemanticFrontier::SemanticFrontier() : Node("cluster_node") 
{
    // Constructor implementation
    RCLCPP_INFO(this->get_logger(), "Semantic Frontier Exploration Node has been initialized.");

    getParameters();

    paramCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&SemanticFrontier::onParameterChange, this, std::placeholders::_1)
    );

    // Define subscribers
    occupancygridSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,std::bind(&SemanticFrontier::occupancyGridCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "%s", (valueMapNamespace + "/value_map_raw").c_str());

    valueMapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(valueMapNamespace + "/value_map_raw", 10,std::bind(&SemanticFrontier::valueMapCallback, this, std::placeholders::_1));

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
    this->declare_parameter("publish_frequency", 1.0);

    this->get_parameter("publish_frequency", publishFrequency);

    this->declare_parameter("value_map_namespace", "/value_map");

    this->get_parameter("value_map_namespace", valueMapNamespace);

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
    if(occGrid.get() == nullptr) 
    {
        RCLCPP_WARN(this->get_logger(), "Occupancy grid is not available yet.");
        return;
    }

    // Process the occupancy grid and detect all frontiers/points
    nav_msgs::msg::OccupancyGrid frontierGrid = frontiers->detectFrontierGrid(occGrid);

    auto frontierGridPtr = std::make_shared<nav_msgs::msg::OccupancyGrid>(frontierGrid);
    std::vector<Frontier> clusteredFrontiers = frontiers->clusterFrontierGrid(frontierGridPtr);

    graphNodes->clear();
    int i = 0;
    for (auto& frontier : clusteredFrontiers)
    {
        int id = i++;
        GraphNode graphNode;
        geometry_msgs::msg::Point centroid = frontier.getCentroid();
        graphNode.setPosition(centroid);
        double score = getScoreFromValueMap(valueMapPcl, centroid);
        if (score < 0.005) continue;  // Skip frontiers with low scores
        graphNode.setScore(score);
        graphNode.setId(id);
        graphNodes->addNode(graphNode);
    }

    graphNodes->publishPosMarkers();

    graphNodes->publishGraphNodeArray();

    // frontiers->publishFrontiers(scoredFrontiers);

    // frontiers->publishGraphNodes(scoredFrontiers);
}

float SemanticFrontier::getScoreFromValueMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const geometry_msgs::msg::Point& pos)
{
    if (!cloud || cloud->empty()) return 0.0f;

    float radius = 0.3;  // Meter
    float radius_squared = radius * radius;
    float best_score = 0.0f;
    bool found = false;

    for (const auto& pt : cloud->points)
    {
        float dx = pt.x - pos.x;
        float dy = pt.y - pos.y;
        float dz = pt.z - pos.z;

        float dist_sq = dx * dx + dy * dy + dz * dz;

        if (dist_sq < radius_squared)
        {
            best_score = std::max(best_score, pt.intensity);
            found = true;
        }
    }

    return found ? best_score : 0.0f;
}

