#include "value_map.hpp"

ValueMap::ValueMap(const rclcpp::NodeOptions & options)
: LifecycleNode("value_map_node", options)
{
    RCLCPP_INFO(this->get_logger(), "%s[Lifecycle]%s ValueMap Node %sinitialized%s.",
                BLUE, RESET, GREEN, RESET);

    // Declare parameters
    declare_parameter<std::string>("vlm_target_namespace", "/seem_ros");

    semanticMap = std::make_unique<SemanticValueMap>(this);
    serviceHandler = std::make_unique<ServiceHandler>(this);
    robot = std::make_unique<Robot>(this);
}

ValueMap::~ValueMap() 
{
    RCLCPP_INFO(this->get_logger(), "%s[Lifecycle]%s ValueMap Node %sdestructed%s.",
                BLUE, RESET, YELLOW, RESET);
}

CallbackReturn ValueMap::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Configuring ValueMap...", BLUE, RESET);

    if(!semanticMap->on_configure()) 
    {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Could not %sconfigure%s Node.", BLUE, RESET, RED, RESET);
        return CallbackReturn::FAILURE;
    }

    if(!serviceHandler->on_configure())
    {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Could not %sconfigure%s ServiceHandler.", BLUE, RESET, RED, RESET);
        return CallbackReturn::FAILURE;
    }

    if(!robot->on_configure())
    {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Could not %sconfigure%s Robot.", BLUE, RESET, RED, RESET);
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;

}

CallbackReturn ValueMap::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Activating ValueMap...", BLUE, RESET);

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),    
        std::bind(&ValueMap::timerCallback, this)
    );

    if (!semanticMap->on_activate()) 
    {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Could not %sactivate%s Node.", BLUE, RESET, RED, RESET);
        return CallbackReturn::FAILURE;
    }

    if (!serviceHandler->on_activate())
    {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Could not %sactivate%s ServiceHandler.", BLUE, RESET, RED, RESET);
        return CallbackReturn::FAILURE;
    }

    if (!robot->on_activate())
    {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Could not %sactivate%s Robot.", BLUE, RESET, RED, RESET);
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ValueMap::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Deactivating ValueMap...", BLUE, RESET);
    semanticMap->on_deactivate();
    serviceHandler->on_deactivate();
    robot->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ValueMap::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Cleaning up ValueMap...", BLUE, RESET);
    if (timer){ timer.reset();}

    semanticMap->on_cleanup();
    serviceHandler->on_cleanup();
    robot->on_cleanup();

    RCLCPP_INFO(this->get_logger(), "%s[Cleanup]%s Timer and subscriber cleaned up.", BLUE, RESET);
    return CallbackReturn::SUCCESS;
}

CallbackReturn ValueMap::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Shutting down ValueMap...", BLUE, RESET);
    semanticMap->on_shutdown();
    serviceHandler->on_shutdown();
    robot->on_shutdown();
    return CallbackReturn::SUCCESS;
}

void ValueMap::timerCallback()
{
    // Get image
    sensor_msgs::msg::Image::SharedPtr rgbImage = robot->getImage();

    // If no image is available, return
    if (!rgbImage) return;

    // Get pose
    geometry_msgs::msg::PoseStamped::SharedPtr pose = robot->getPose(rgbImage->header.stamp);

    // If pose is empty, return
    if (!pose) return;

    // Get cosine similarity score
    double score = serviceHandler->getSemanticSimilarityScore(*rgbImage, "chair");
    semScore.setScore(score);
    semScore.setHeader(rgbImage->header);

    // Update semantic map
    semanticMap->updateSemanticMap(semScore, *pose);

}
