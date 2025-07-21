#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("image_subscriber_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb", 10,
            std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Image subscriber node started, listening on /rgb");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
