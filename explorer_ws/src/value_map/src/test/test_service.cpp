#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "seem_ros_interfaces/srv/panoptic.hpp"
#include "seem_ros_interfaces/srv/object_segmentation.hpp"
#include "seem_ros_interfaces/srv/semantic_similarity.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ServiceHandler
{
public:
    ServiceHandler(rclcpp::Node::SharedPtr node)
        : node_(node), bridge_(std::make_shared<cv_bridge::CvImage>())
    {
        vlm_namespace_ = node_->declare_parameter<std::string>("vlm_target_namespace", "/seem_ros");
        if (!vlm_namespace_.empty() && vlm_namespace_.back() == '/')
            vlm_namespace_.pop_back();

        auto resolve = [this](const std::string &service) {
            return vlm_namespace_ + "/" + service;
        };

        cli_panoptic_ = node_->create_client<seem_ros_interfaces::srv::Panoptic>(resolve("panoptic_segmentation"));
        cli_object_ = node_->create_client<seem_ros_interfaces::srv::ObjectSegmentation>(resolve("object_segmentation"));
        cli_similarity_ = node_->create_client<seem_ros_interfaces::srv::SemanticSimilarity>(resolve("semantic_similarity"));

        subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/rgb", 10, std::bind(&ServiceHandler::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "ServiceHandler initialized with VLM namespace: %s", vlm_namespace_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Received image, calling all services...");

        call_panoptic(*msg);
        call_object_segmentation(*msg, "bottle");
        call_semantic_similarity(*msg, "bottle");
    }

    void call_panoptic(const sensor_msgs::msg::Image &image_msg)
    {
        auto request = std::make_shared<seem_ros_interfaces::srv::Panoptic::Request>();
        request->image = image_msg;

        cli_panoptic_->async_send_request(request,
            [this](rclcpp::Client<seem_ros_interfaces::srv::Panoptic>::SharedFuture future) {
                try {
                    auto response = future.get();
                    cv::Mat img = cv_bridge::toCvCopy(response->panoptic_segmentation, "rgb8")->image;
                    cv::imshow("Panoptic Segmentation", img);
                    cv::waitKey(1);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(node_->get_logger(), "Panoptic response error: %s", e.what());
                }
            });
    }

    void call_object_segmentation(const sensor_msgs::msg::Image &image_msg, const std::string &query)
    {
        auto request = std::make_shared<seem_ros_interfaces::srv::ObjectSegmentation::Request>();
        request->image = image_msg;
        request->query = query;

        cli_object_->async_send_request(request,
            [this](rclcpp::Client<seem_ros_interfaces::srv::ObjectSegmentation>::SharedFuture future) {
                try {
                    auto response = future.get();
                    cv::Mat img = cv_bridge::toCvCopy(response->segmented_image, "rgb8")->image;
                    cv::imshow("Object Segmentation", img);
                    cv::waitKey(1);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(node_->get_logger(), "Object response error: %s", e.what());
                }
            });
    }

    void call_semantic_similarity(const sensor_msgs::msg::Image &image_msg, const std::string &query)
    {
        auto request = std::make_shared<seem_ros_interfaces::srv::SemanticSimilarity::Request>();
        request->image = image_msg;
        request->query = query;

        cli_similarity_->async_send_request(request,
            [this](rclcpp::Client<seem_ros_interfaces::srv::SemanticSimilarity>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(node_->get_logger(), "Semantic similarity score: %.3f", response->score);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(node_->get_logger(), "Similarity response error: %s", e.what());
                }
            });
    }

    rclcpp::Node::SharedPtr node_;
    std::string vlm_namespace_;
    std::shared_ptr<cv_bridge::CvImage> bridge_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    rclcpp::Client<seem_ros_interfaces::srv::Panoptic>::SharedPtr cli_panoptic_;
    rclcpp::Client<seem_ros_interfaces::srv::ObjectSegmentation>::SharedPtr cli_object_;
    rclcpp::Client<seem_ros_interfaces::srv::SemanticSimilarity>::SharedPtr cli_similarity_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_service_node");
    auto handler = std::make_shared<ServiceHandler>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
