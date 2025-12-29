#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <mutex>
#include <string>

class OverlayVideoRecorder
{
public:
    explicit OverlayVideoRecorder(
        const rclcpp::Node::SharedPtr& node,
        const std::string& topic = "/yoloe/overlay");

    void start(const std::string& outputPath);
    void stop();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void writeFrame();

private:
    rclcpp::Node::SharedPtr node_;
    std::string topic_;
    std::string outputPath_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoWriter videoWriter_;
    cv::Mat latestFrame_;

    std::mutex mutex_;
};
