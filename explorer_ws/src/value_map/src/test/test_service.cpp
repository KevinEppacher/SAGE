#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "seem_ros_interfaces/srv/panoptic.hpp"
#include "seem_ros_interfaces/srv/object_segmentation.hpp"
#include "seem_ros_interfaces/srv/semantic_similarity.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "seem_ros_interfaces/srv/panoptic.hpp"
#include "seem_ros_interfaces/srv/object_segmentation.hpp"
#include "seem_ros_interfaces/srv/semantic_similarity.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using rclcpp_lifecycle::LifecycleNode;

class ServiceHandlerNode : public LifecycleNode
{
public:
    explicit ServiceHandlerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : LifecycleNode("service_handler_node", options)
    {
        declare_parameter<std::string>("vlm_target_namespace", "/seem_ros");
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        vlm_namespace_ = get_parameter("vlm_target_namespace").as_string();
        if (!vlm_namespace_.empty() && vlm_namespace_.back() == '/')
            vlm_namespace_.pop_back();

        auto resolve = [this](const std::string &srv) {
            return vlm_namespace_ + "/" + srv;
        };

        cli_panoptic_ = create_client<seem_ros_interfaces::srv::Panoptic>(resolve("panoptic_segmentation"));
        cli_object_ = create_client<seem_ros_interfaces::srv::ObjectSegmentation>(resolve("object_segmentation"));
        cli_similarity_ = create_client<seem_ros_interfaces::srv::SemanticSimilarity>(resolve("semantic_similarity"));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/rgb", 10,
            std::bind(&ServiceHandlerNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Activated and ready to receive images.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_WARN(get_logger(), "Ignoring image; node not active.");
            return;
        }

        call_panoptic(*msg);
        call_object_segmentation(*msg, "bottle");
        call_semantic_similarity(*msg, "bottle");
    }

    void call_panoptic(const sensor_msgs::msg::Image &image_msg)
    {
        auto req = std::make_shared<seem_ros_interfaces::srv::Panoptic::Request>();
        req->image = image_msg;

        cli_panoptic_->async_send_request(req,
            [this](rclcpp::Client<seem_ros_interfaces::srv::Panoptic>::SharedFuture future) {
                try {
                    auto res = future.get();
                    cv::Mat img = cv_bridge::toCvCopy(res->panoptic_segmentation, "rgb8")->image;
                    cv::imshow("Panoptic Segmentation", img);
                    cv::waitKey(1);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Panoptic response error: %s", e.what());
                }
            });
    }

    void call_object_segmentation(const sensor_msgs::msg::Image &image_msg, const std::string &query)
    {
        auto req = std::make_shared<seem_ros_interfaces::srv::ObjectSegmentation::Request>();
        req->image = image_msg;
        req->query = query;

        cli_object_->async_send_request(req,
            [this](rclcpp::Client<seem_ros_interfaces::srv::ObjectSegmentation>::SharedFuture future) {
                try {
                    auto res = future.get();
                    cv::Mat img = cv_bridge::toCvCopy(res->segmented_image, "rgb8")->image;
                    cv::imshow("Object Segmentation", img);
                    cv::waitKey(1);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "ObjectSegmentation response error: %s", e.what());
                }
            });
    }

    void call_semantic_similarity(const sensor_msgs::msg::Image &image_msg, const std::string &query)
    {
        auto req = std::make_shared<seem_ros_interfaces::srv::SemanticSimilarity::Request>();
        req->image = image_msg;
        req->query = query;

        cli_similarity_->async_send_request(req,
            [this](rclcpp::Client<seem_ros_interfaces::srv::SemanticSimilarity>::SharedFuture future) {
                try {
                    auto res = future.get();
                    RCLCPP_INFO(this->get_logger(), "Semantic similarity score: %.3f", res->score);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "SemanticSimilarity response error: %s", e.what());
                }
            });
    }

    std::string vlm_namespace_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Client<seem_ros_interfaces::srv::Panoptic>::SharedPtr cli_panoptic_;
    rclcpp::Client<seem_ros_interfaces::srv::ObjectSegmentation>::SharedPtr cli_object_;
    rclcpp::Client<seem_ros_interfaces::srv::SemanticSimilarity>::SharedPtr cli_similarity_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceHandlerNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    // Manuell durch Lifecycle-Stufen
    node->configure();
    node->activate();

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
