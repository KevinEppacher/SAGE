#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class RGBImageSubscriber : public rclcpp::Node
{
public:
    RGBImageSubscriber()
    : Node("rgb_image_subscriber")
    {
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();
        qos_profile.keep_last(10);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb",
            rclcpp::SensorDataQoS(),
            std::bind(&RGBImageSubscriber::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "RGB Image Subscriber started with Best Effort QoS.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::imshow("RGB Image", cv_image);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CvBridge error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
