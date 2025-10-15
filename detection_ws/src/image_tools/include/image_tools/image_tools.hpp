#ifndef IMAGE_TOOLS_HPP
#define IMAGE_TOOLS_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageTools : public rclcpp::Node
{
public:
    ImageTools();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void timerCallback();
    rclcpp::QoS createQoS(const std::string &prefix);
    cv::Mat resizeImage(const cv::Mat &input);
    bool hasSignificantChange(const cv::Mat &current);

    // Parameters
    std::string inputTopic;
    std::string outputTopic;
    double timerFrequency;
    bool resizeEnabled;
    int resizeWidth;
    int resizeHeight;
    bool changeGateEnabled;      // <--- new
    double histogramThreshold;

    // ROS entities
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::Image::SharedPtr latestImage;

    // Previous frame histogram
    cv::Mat previousHist;
    bool hasPreviousHist = false;
};

#endif  // IMAGE_TOOLS_HPP
