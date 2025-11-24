#include "value_map.hpp"

ValueMap::ValueMap(const rclcpp::NodeOptions & options)
: LifecycleNode("value_map_node", options)
{
    RCLCPP_INFO(this->get_logger(), "%s[Lifecycle]%s ValueMap Node %sinitialized%s.",
                BLUE, RESET, GREEN, RESET);

    // Declare parameters
    declare_parameter<std::string>("vlm_target_namespace", "/blip2_ros");

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

    this->declare_parameter<double>("timer_frequency", 5.0);
    this->declare_parameter<std::string>("semantic_prompt_topic", "/user_prompt");

    this->get_parameter("timer_frequency", timerFrequency);
    this->get_parameter("semantic_prompt_topic", semanticPromptTopic);

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

    semanticPromptSub = this->create_subscription<multimodal_query_msgs::msg::SemanticPrompt>(
        semanticPromptTopic, 10, std::bind(&ValueMap::semanticPromptCallback, this, std::placeholders::_1));

    cosineSimPub = this->create_publisher<std_msgs::msg::Float32>("cosine_similarity", 10);

    return CallbackReturn::SUCCESS;

}

CallbackReturn ValueMap::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Activating ValueMap...", BLUE, RESET);

    auto interval = std::chrono::duration<double>(1.0 / timerFrequency);
    RCLCPP_INFO(this->get_logger(), "Publishing with a time interval of %.3f seconds", interval.count());
    timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(interval),    
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
    cosineSimPub.reset();
    semanticPromptSub.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ValueMap::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Cleaning up ValueMap...", BLUE, RESET);
    if (timer){ timer.reset();}

    semanticMap->on_cleanup();
    serviceHandler->on_cleanup();
    robot->on_cleanup();
    cosineSimPub.reset();
    semanticPromptSub.reset();

    RCLCPP_INFO(this->get_logger(), "%s[Cleanup]%s Timer and subscriber cleaned up.", BLUE, RESET);
    return CallbackReturn::SUCCESS;
}

CallbackReturn ValueMap::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "%s[Transition]%s Shutting down ValueMap...", BLUE, RESET);
    semanticMap->on_shutdown();
    serviceHandler->on_shutdown();
    robot->on_shutdown();
    cosineSimPub.reset();
    semanticPromptSub.reset();
    return CallbackReturn::SUCCESS;
}

void ValueMap::timerCallback()
{
    /////////////////////////////////////////////////////////////
    // Test Case. Remove when done. Setting hardcoded text prompt.
    // textPrompt = "door";
    /////////////////////////////////////////////////////////////

    const std::string prompt = textPrompt;  // or guard with a mutex if multithreaded
    if (prompt.find_first_not_of(" \t\n\r") == std::string::npos) 
    {
        RCLCPP_INFO(this->get_logger(), "No text prompt received yet.");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Current text prompt: %s", prompt.c_str());

    // Get image
    sensor_msgs::msg::Image::SharedPtr rgbImage = robot->getImage();
    
    // If no image is available, return
    if (!rgbImage) return;

    // Get cosine similarity score
    double score = serviceHandler->getSemanticSimilarityScore(*rgbImage, textPrompt);
    score = std::max(0.0, score);  // Ensure non-negative score
    score = std::min(1.0, score);  // Cap at 1.0

    score = 1.0;        // TEMPORARY OVERRIDE. REMOVE AFTER TESTING.
    
    // Publish cosine similarity score
    publishCosineSimilarity(score);

    // Get pose
    geometry_msgs::msg::PoseStamped::SharedPtr pose = robot->getPose(rgbImage->header.stamp);

    // If pose is empty, return
    if (!pose) return;

    semScore.setScore(score);
    semScore.setHeader(rgbImage->header);

    RCLCPP_DEBUG(this->get_logger(), "%s%s Semantic Similarity: %.2f", BLUE, RESET, score);

    // Update semantic map
    semanticMap->updateSemanticMap(semScore, *pose);

}

void ValueMap::semanticPromptCallback(const multimodal_query_msgs::msg::SemanticPrompt::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Semantic Prompt: %s", msg->text_query.c_str());
    textPrompt = msg->text_query;
    semanticMap->clearValueMap();
}

void ValueMap::publishCosineSimilarity(double score)
{
    auto message = std_msgs::msg::Float32();
    message.data = static_cast<float>(score);
    cosineSimPub->publish(message);
}