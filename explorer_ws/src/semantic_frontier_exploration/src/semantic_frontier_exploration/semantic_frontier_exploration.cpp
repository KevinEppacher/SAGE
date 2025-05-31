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
    occupancygridSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&SemanticFrontier::occupancyGridCallback, this, std::placeholders::_1));

    // Define timers
    auto interval = std::chrono::duration<double>(1.0 / publishFrequency);
    RCLCPP_INFO(this->get_logger(), "Publishing with a time interval of %.3f seconds", interval.count());
    timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(interval),
        std::bind(&SemanticFrontier::timerCallback, this)
    );

    frontiers = std::make_shared<FrontierCollection>(this);

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

void SemanticFrontier::getParameters()
{
    this->declare_parameter("publish_frequency", 1.0);

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

    frontiers->publishFrontiers(clusteredFrontiers);

    

}

