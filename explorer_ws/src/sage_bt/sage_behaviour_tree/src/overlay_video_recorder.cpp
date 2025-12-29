#include "sage_behaviour_tree/overlay_video_recorder.hpp"

#include <cv_bridge/cv_bridge.h>

#include <chrono>

OverlayVideoRecorder::OverlayVideoRecorder(
    const rclcpp::Node::SharedPtr& node,
    const std::string& topic)
    : node_(node),
      topic_(topic)
{
    rclcpp::QoS qos(1);
    qos.best_effort();
    qos.durability_volatile();

    subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        topic_,
        qos,
        std::bind(
            &OverlayVideoRecorder::imageCallback,
            this,
            std::placeholders::_1));
}

void OverlayVideoRecorder::start(const std::string& outputPath)
{
    std::lock_guard<std::mutex> lock(mutex_);

    outputPath_ = outputPath;
    videoWriter_.release();
    latestFrame_.release();

    timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&OverlayVideoRecorder::writeFrame, this));

    RCLCPP_INFO(
        node_->get_logger(),
        "[OverlayVideoRecorder] Recording started → %s",
        outputPath_.c_str());
}

void OverlayVideoRecorder::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (timer_)
    {
        timer_->cancel();
        timer_.reset();
    }

    if (videoWriter_.isOpened())
    {
        videoWriter_.release();
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "[OverlayVideoRecorder] Recording stopped");
}

void OverlayVideoRecorder::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        std::lock_guard<std::mutex> lock(mutex_);
        latestFrame_ = frame.clone();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Overlay image conversion failed: %s",
            e.what());
    }
}

void OverlayVideoRecorder::writeFrame()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (latestFrame_.empty())
        return;

    if (!videoWriter_.isOpened())
    {
        const int width  = latestFrame_.cols;
        const int height = latestFrame_.rows;

        videoWriter_.open(
            outputPath_,
            cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
            1.0,
            cv::Size(width, height),
            true);

        if (!videoWriter_.isOpened())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to open VideoWriter for %s",
                outputPath_.c_str());
            return;
        }
    }

    videoWriter_.write(latestFrame_);
}
