#include "image_tools.hpp"

ImageTools::ImageTools()
: Node("image_tools")
{
    // --- Parameters ---
    this->declare_parameter<std::string>("input_topic", "/rgb");
    this->declare_parameter<std::string>("output_topic", "/rgb/throttled");
    this->declare_parameter<double>("timer_frequency", 10.0);
    this->declare_parameter<bool>("resize_enabled", false);
    this->declare_parameter<int>("resize_width", 640);
    this->declare_parameter<int>("resize_height", 480);

    // QoS parameters
    this->declare_parameter<std::string>("input_qos.reliability", "best_effort");
    this->declare_parameter<std::string>("input_qos.durability", "volatile");
    this->declare_parameter<std::string>("input_qos.history", "keep_last");
    this->declare_parameter<int>("input_qos.depth", 10);
    this->declare_parameter<std::string>("output_qos.reliability", "best_effort");
    this->declare_parameter<std::string>("output_qos.durability", "volatile");
    this->declare_parameter<std::string>("output_qos.history", "keep_last");
    this->declare_parameter<int>("output_qos.depth", 10);

    // --- Read parameters ---
    this->get_parameter("input_topic", inputTopic);
    this->get_parameter("output_topic", outputTopic);
    this->get_parameter("timer_frequency", timerFrequency);
    this->get_parameter("resize_enabled", resizeEnabled);
    this->get_parameter("resize_width", resizeWidth);
    this->get_parameter("resize_height", resizeHeight);

    // --- Setup QoS ---
    rclcpp::QoS inputQos = createQoS("input_qos");
    rclcpp::QoS outputQos = createQoS("output_qos");

    subscription = this->create_subscription<sensor_msgs::msg::Image>(
        inputTopic, inputQos,
        std::bind(&ImageTools::imageCallback, this, std::placeholders::_1));

    publisher = this->create_publisher<sensor_msgs::msg::Image>(outputTopic, outputQos);

    // --- Timer ---
    auto interval = std::chrono::duration<double>(1.0 / timerFrequency);
    timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(interval),
        std::bind(&ImageTools::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
        "ImageTools node started. Input: %s | Output: %s | Hz: %.2f | Resize: %s (%dx%d)",
        inputTopic.c_str(), outputTopic.c_str(), timerFrequency,
        resizeEnabled ? "enabled" : "disabled", resizeWidth, resizeHeight);
}

rclcpp::QoS ImageTools::createQoS(const std::string &prefix)
{
    std::string reliability = "best_effort";
    std::string durability = "volatile";
    std::string history = "keep_last";
    int depth = 10;

    this->get_parameter_or(prefix + ".reliability", reliability, reliability);
    this->get_parameter_or(prefix + ".durability", durability, durability);
    this->get_parameter_or(prefix + ".history", history, history);
    this->get_parameter_or(prefix + ".depth", depth, depth);

    rmw_qos_reliability_policy_t rel_policy =
        (reliability == "reliable") ? RMW_QOS_POLICY_RELIABILITY_RELIABLE :
                                      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_qos_durability_policy_t dur_policy =
        (durability == "transient_local") ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL :
                                            RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rmw_qos_history_policy_t hist_policy =
        (history == "keep_all") ? RMW_QOS_POLICY_HISTORY_KEEP_ALL :
                                  RMW_QOS_POLICY_HISTORY_KEEP_LAST;

    rclcpp::QoS qos(rclcpp::QoSInitialization(hist_policy, depth));
    qos.reliability(rel_policy);
    qos.durability(dur_policy);
    return qos;
}

void ImageTools::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    latestImage = msg;
}

cv::Mat ImageTools::resizeImage(const cv::Mat &input)
{
    if (!resizeEnabled)
        return input;
    cv::Mat output;
    cv::resize(input, output, cv::Size(resizeWidth, resizeHeight));
    return output;
}

bool ImageTools::isDepthImage(const std::string &encoding)
{
    return (encoding == "16UC1" || encoding == "32FC1");
}

void ImageTools::checkTimerRate()
{
    rclcpp::Time now = this->now();
    if (firstCallback) {
        firstCallback = false;
        lastCallbackTime = now;
        return;
    }

    double elapsed = (now - lastCallbackTime).seconds();
    double expected = 1.0 / timerFrequency;
    lastCallbackTime = now;

    if (std::abs(elapsed - expected) > expected * 0.2) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Timer drift: actual %.4fs vs expected %.4fs (%.1f%% off)",
            elapsed, expected, ((elapsed - expected) / expected) * 100.0);
    }
}

void ImageTools::timerCallback()
{
    checkTimerRate();

    if (!latestImage)
        return;

    try {
        const std::string encoding = latestImage->encoding;
        isDepthImage(encoding);

        cv::Mat cv_image = cv_bridge::toCvShare(latestImage, encoding)->image;
        cv::Mat processed = resizeImage(cv_image);

        auto header = latestImage->header;
        auto out = cv_bridge::CvImage(header, encoding, processed).toImageMsg();
        publisher->publish(*out);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "CvBridge error: %s", e.what());
    }
}
