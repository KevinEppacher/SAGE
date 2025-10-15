#include "image_tools.hpp"

ImageTools::ImageTools()
: Node("image_tools")
{
    // --- Declare parameters ---
    this->declare_parameter<std::string>("input_topic", "/rgb");
    this->declare_parameter<std::string>("output_topic", "/rgb_throttled");
    this->declare_parameter<double>("timer_frequency", 10.0);
    this->declare_parameter<bool>("resize_enabled", false);
    this->declare_parameter<int>("resize_width", 640);
    this->declare_parameter<int>("resize_height", 480);
    this->declare_parameter<bool>("change_gate_enabled", true);   // <--- new
    this->declare_parameter<double>("histogram_threshold", 0.90);

    // QoS parameters (same as before) ...
    this->declare_parameter<std::string>("input_qos.reliability", "best_effort");
    this->declare_parameter<std::string>("input_qos.durability", "volatile");
    this->declare_parameter<std::string>("input_qos.history", "keep_last");
    this->declare_parameter<int>("input_qos.depth", 10);
    this->declare_parameter<std::string>("output_qos.reliability", "best_effort");
    this->declare_parameter<std::string>("output_qos.durability", "volatile");
    this->declare_parameter<std::string>("output_qos.history", "keep_last");
    this->declare_parameter<int>("output_qos.depth", 10);

    // --- Get parameters ---
    this->get_parameter("input_topic", inputTopic);
    this->get_parameter("output_topic", outputTopic);
    this->get_parameter("timer_frequency", timerFrequency);
    this->get_parameter("resize_enabled", resizeEnabled);
    this->get_parameter("resize_width", resizeWidth);
    this->get_parameter("resize_height", resizeHeight);
    this->get_parameter("change_gate_enabled", changeGateEnabled);  // <--- new
    this->get_parameter("histogram_threshold", histogramThreshold);

    // QoS creation and entities (unchanged) ...
    rclcpp::QoS inputQos = createQoS("input_qos");
    rclcpp::QoS outputQos = createQoS("output_qos");
    subscription = this->create_subscription<sensor_msgs::msg::Image>(
        inputTopic, inputQos, std::bind(&ImageTools::imageCallback, this, std::placeholders::_1));
    publisher = this->create_publisher<sensor_msgs::msg::Image>(outputTopic, outputQos);

    auto period = std::chrono::duration<double>(1.0 / timerFrequency);
    timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ImageTools::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
        "ImageTools node started. Input: %s | Output: %s | Hz: %.2f | Resize: %s (%dx%d) | Gate: %s | Threshold: %.2f",
        inputTopic.c_str(), outputTopic.c_str(), timerFrequency,
        resizeEnabled ? "enabled" : "disabled", resizeWidth, resizeHeight,
        changeGateEnabled ? "enabled" : "disabled", histogramThreshold);
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
    if (!resizeEnabled) return input;
    cv::Mat output;
    cv::resize(input, output, cv::Size(resizeWidth, resizeHeight));
    return output;
}

bool ImageTools::hasSignificantChange(const cv::Mat &current)
{
    if (current.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty frame received, skipping histogram check.");
        return false;
    }

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(current, gray, cv::COLOR_BGR2GRAY);

    // Compute histogram
    int histSize = 256;
    float range[] = {0, 256};
    const float *histRange = {range};
    cv::Mat hist;
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    if (hist.empty()) {
        RCLCPP_WARN(this->get_logger(), "Histogram computation failed.");
        return false;
    }

    // Normalize
    cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX);

    // First frame always considered significant
    if (!hasPreviousHist) {
        previousHist = hist.clone();
        hasPreviousHist = true;
        RCLCPP_INFO(this->get_logger(), "First frame -> publishing.");
        return true;
    }

    // Compare histograms
    double diff = cv::compareHist(previousHist, hist, cv::HISTCMP_CHISQR);

    RCLCPP_INFO(this->get_logger(), "Histogram diff (Chi2): %.5f (threshold %.3f)", diff, histogramThreshold);

    previousHist = hist.clone();

    bool changed = diff > histogramThreshold;
    if (!changed) {
        RCLCPP_INFO(this->get_logger(), "Image skipped (diff %.4f >= threshold %.2f)",
                     diff, histogramThreshold);
    } else {
        RCLCPP_INFO(this->get_logger(), "Image changed (diff %.4f < threshold %.2f)",
                     diff, histogramThreshold);
    }
    return changed;
}

void ImageTools::timerCallback()
{
    if (!latestImage) return;

    try {
        cv::Mat cv_image = cv_bridge::toCvShare(latestImage, "bgr8")->image;
        cv::Mat processed = resizeImage(cv_image);

        // Conditional gating
        if (changeGateEnabled && !hasSignificantChange(processed))
            return;

        auto header = latestImage->header;
        auto out = cv_bridge::CvImage(header, "bgr8", processed).toImageMsg();
        publisher->publish(*out);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "CvBridge error: %s", e.what());
    }
}
