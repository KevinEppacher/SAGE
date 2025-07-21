#include "value_map.hpp"

ValueMap::ValueMap(const rclcpp::NodeOptions & options)
: LifecycleNode("value_map_node", options)
{
    RCLCPP_INFO(this->get_logger(), "%s[Lifecycle]%s ValueMap Node %sinitialized%s.",
                BLUE, RESET, GREEN, RESET);

    semanticMap = std::make_unique<SemanticValueMap>(this);
}

ValueMap::~ValueMap() 
{
    RCLCPP_INFO(this->get_logger(), "%s[Lifecycle]%s ValueMap Node %sdestructed%s.",
                BLUE, RESET, YELLOW, RESET);
}

CallbackReturn ValueMap::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Configuring ValueMap...", BLUE, RESET);

    if (semanticMap->on_configure()) {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Node %sconfigured%s.", BLUE, RESET, GREEN, RESET);
        return CallbackReturn::SUCCESS;
    }
    return CallbackReturn::FAILURE;
}

CallbackReturn ValueMap::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Activating ValueMap...", BLUE, RESET);

    rgbSub = this->create_subscription<sensor_msgs::msg::Image>(
    "/rgb", 10,
    std::bind(&ValueMap::rgbCallback, this, std::placeholders::_1)
    );

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ValueMap::timerCallback, this)
    );

    if (semanticMap->on_activate()) {
        RCLCPP_INFO(this->get_logger(), "%s[Status]%s Node %sactivated%s.", BLUE, RESET, GREEN, RESET);
        return CallbackReturn::SUCCESS;
    }
    return CallbackReturn::FAILURE;
}

CallbackReturn ValueMap::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Deactivating ValueMap...", BLUE, RESET);
    semanticMap->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ValueMap::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Cleaning up ValueMap...", BLUE, RESET);
    if (timer) {
        timer.reset();
    }
    if (rgbSub) {
        rgbSub.reset();
    }

    semanticMap->on_cleanup();

    RCLCPP_INFO(this->get_logger(), "%s[Cleanup]%s Timer and subscriber cleaned up.", BLUE, RESET);
    return CallbackReturn::SUCCESS;
}

CallbackReturn ValueMap::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Shutting down ValueMap...", BLUE, RESET);
    semanticMap->on_shutdown();
    return CallbackReturn::SUCCESS;
}

void ValueMap::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "%s[Timer]%s Timer callback executed.", BLUE, RESET);

}

void ValueMap::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        // Convert to OpenCV image (BGR8 encoding expected)
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        cv::imshow("RGB Image", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}