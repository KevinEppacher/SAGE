#include "service_handler.hpp"

ServiceHandler::ServiceHandler(rclcpp_lifecycle::LifecycleNode* node) : node(node)
{

}

ServiceHandler::~ServiceHandler()
{

}

bool ServiceHandler::on_configure()
{
    RCLCPP_INFO(node->get_logger(), "Configuring ServiceHandler...");

    vlmNamespace = node->get_parameter("vlm_target_namespace").as_string();
    if (!vlmNamespace.empty() && vlmNamespace.back() == '/')
        vlmNamespace.pop_back();

    auto resolve = [this](const std::string &srv) {
        return vlmNamespace + "/" + srv;
    };

    RCLCPP_INFO(node->get_logger(), "vlm_target_namespace: %s", vlmNamespace.c_str());

    cliPanoptic = node->create_client<seem_ros_interfaces::srv::Panoptic>(resolve("panoptic_segmentation"));
    cliObjSeg = node->create_client<seem_ros_interfaces::srv::ObjectSegmentation>(resolve("object_segmentation"));
    cliSemSim = node->create_client<seem_ros_interfaces::srv::SemanticSimilarity>(resolve("semantic_similarity"));

    RCLCPP_INFO(node->get_logger(), "ServiceHandler configured successfully.");

    return true;
}

bool ServiceHandler::on_activate()
{
    RCLCPP_INFO(node->get_logger(), "Activating ServiceHandler...");

    
    RCLCPP_INFO(node->get_logger(), "ServiceHandler activated successfully.");
    return true;
}

bool ServiceHandler::on_deactivate()
{
    RCLCPP_INFO(node->get_logger(), "Deactivating ServiceHandler...");

    RCLCPP_INFO(node->get_logger(), "ServiceHandler deactivated successfully.");
    return true;
}

bool ServiceHandler::on_cleanup()
{
    RCLCPP_INFO(node->get_logger(), "Cleaning up ServiceHandler...");

    RCLCPP_INFO(node->get_logger(), "ServiceHandler cleaned up successfully.");
    return true;
}

bool ServiceHandler::on_shutdown()
{
    RCLCPP_INFO(node->get_logger(), "Shutting down ServiceHandler...");

    RCLCPP_INFO(node->get_logger(), "ServiceHandler shut down successfully.");
    return true;
}

void ServiceHandler::callPanoptic(const sensor_msgs::msg::Image &image_msg)
{
    auto req = std::make_shared<seem_ros_interfaces::srv::Panoptic::Request>();
    req->image = image_msg;

    cliPanoptic->async_send_request(req,
        [this](rclcpp::Client<seem_ros_interfaces::srv::Panoptic>::SharedFuture future) {
            try {
                auto res = future.get();
                panopticSegmentedImage = res->panoptic_segmentation;
                RCLCPP_INFO(node->get_logger(), "Received panoptic segmentation response: %dx%d", 
                           panopticSegmentedImage.width, panopticSegmentedImage.height);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node->get_logger(), "Panoptic response error: %s", e.what());
            }
        });
}

void ServiceHandler::callObjectSegmentation(const sensor_msgs::msg::Image &image_msg, const std::string &query)
{
    auto req = std::make_shared<seem_ros_interfaces::srv::ObjectSegmentation::Request>();
    req->image = image_msg;
    req->query = query;

    cliObjSeg->async_send_request(req,
        [this](rclcpp::Client<seem_ros_interfaces::srv::ObjectSegmentation>::SharedFuture future) {
            try {
                auto res = future.get();
                objectSegmentedImage = res->segmented_image;
                RCLCPP_INFO(node->get_logger(), "Received object segmentation response: %dx%d", 
                           objectSegmentedImage.width, objectSegmentedImage.height);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node->get_logger(), "ObjectSegmentation response error: %s", e.what());
            }
        });
}

void ServiceHandler::callSemanticSimilarity(const sensor_msgs::msg::Image &image_msg, const std::string &query)
{
    auto req = std::make_shared<seem_ros_interfaces::srv::SemanticSimilarity::Request>();
    req->image = image_msg;
    req->query = query;

    cliSemSim->async_send_request(req,
        [this](rclcpp::Client<seem_ros_interfaces::srv::SemanticSimilarity>::SharedFuture future) {
            try {
                auto res = future.get();
                semanticSimilarityScore = res->score;
                RCLCPP_INFO(node->get_logger(), "Semantic similarity score: %.3f", semanticSimilarityScore);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node->get_logger(), "SemanticSimilarity response error: %s", e.what());
            }
        });
}

sensor_msgs::msg::Image ServiceHandler::getPanopticSegmentedImage(const sensor_msgs::msg::Image &image_msg)
{
    callPanoptic(image_msg);
    return panopticSegmentedImage;
}

sensor_msgs::msg::Image ServiceHandler::getObjectSegmentedImage(const sensor_msgs::msg::Image &image_msg, const std::string &query)
{
    callObjectSegmentation(image_msg, query);
    return objectSegmentedImage;
}

double ServiceHandler::getSemanticSimilarityScore(const sensor_msgs::msg::Image &image_msg, const std::string &query)
{
    callSemanticSimilarity(image_msg, query);
    return semanticSimilarityScore;
}