#include "relevance_map.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

float normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

RelevanceMapNode::RelevanceMapNode(const rclcpp::NodeOptions &options)
: Node("relevance_map_node", options)
{
    // Declare parameters
    this->declare_parameter("input_map_topic", "/map");
    this->declare_parameter("input_graph_topic", "/exploration_graph_nodes/graph_nodes");
    this->declare_parameter("output_graph_topic", "/filtered/exploration_graph_nodes/graph_nodes");
    this->declare_parameter("camera_info_topic", "/camera_info");
    this->declare_parameter("frame_map", "map");
    this->declare_parameter("frame_robot", "base_link");
    this->declare_parameter("debug_mode", false);
    this->declare_parameter("raytracing_enabled", true);
    this->declare_parameter("prompt_topic", "/user_prompt");
    
    using FloatingRange = rcl_interfaces::msg::FloatingPointRange;
    using ParamDesc = rcl_interfaces::msg::ParameterDescriptor;
    
    // --- Slider Parameter mit Ranges deklarieren --- //
    ParamDesc rateDesc;
    rateDesc.description = "Base increment rate (alpha)";
    rateDesc.floating_point_range = {FloatingRange().set__from_value(0.0).set__to_value(0.5).set__step(0.0001)};
    this->declare_parameter("base_increment_rate", 0.1, rateDesc);

    ParamDesc decayDesc;
    decayDesc.description = "Decay factor for relevance retention";
    decayDesc.floating_point_range = {FloatingRange().set__from_value(0.8).set__to_value(1.0).set__step(0.00005)};
    this->declare_parameter("decay_factor", 0.99, decayDesc);
    
    ParamDesc rangeDesc;
    rangeDesc.description = "Maximum range in meters";
    rangeDesc.floating_point_range = {FloatingRange().set__from_value(2.0).set__to_value(20.0).set__step(0.5)};
    this->declare_parameter("max_range", 10.0, rangeDesc);
    
    ParamDesc fovDesc;
    fovDesc.description = "Field of view in degrees";
    fovDesc.floating_point_range = {FloatingRange().set__from_value(10.0).set__to_value(120.0).set__step(1.0)};
    this->declare_parameter("fov_deg", 70.0, fovDesc);
    
    ParamDesc threshDesc;
    threshDesc.description = "Relevance filtering threshold";
    threshDesc.floating_point_range = {FloatingRange().set__from_value(0.0).set__to_value(1.0).set__step(0.05)};
    this->declare_parameter("relevance_threshold", 0.5, threshDesc);
    
    ParamDesc angularSharpDesc;
    angularSharpDesc.description = "Angular confidence sharpness";
    angularSharpDesc.floating_point_range = {FloatingRange().set__from_value(0.0).set__to_value(1.0).set__step(0.05)};
    this->declare_parameter("angular_confidence_sharpness", 0.25, angularSharpDesc);

    ParamDesc radialSharpnessDesc;
    radialSharpnessDesc.description = "Radial confidence sharpness";
    radialSharpnessDesc.floating_point_range = {FloatingRange().set__from_value(0.0).set__to_value(1.0).set__step(0.05)};
    this->declare_parameter("radial_confidence_sharpness", 0.5, radialSharpnessDesc);

    ParamDesc ignoreHighScoreDesc;
    ignoreHighScoreDesc.description = "Score threshold above which graph nodes are ignored for relevance filtering and directly republished";
    ignoreHighScoreDesc.floating_point_range = {
        FloatingRange().set__from_value(0.0).set__to_value(1.0).set__step(0.01)};
    this->declare_parameter("ignore_high_score_threshold", 0.8, ignoreHighScoreDesc);


    // Load parameters
    this->get_parameter("input_map_topic", inputMapTopic);
    this->get_parameter("input_graph_topic", inputGraphTopic);
    this->get_parameter("output_graph_topic", outputGraphTopic);
    this->get_parameter("camera_info_topic", cameraInfoTopic);
    
    this->get_parameter("frame_map", frameMap);
    this->get_parameter("frame_robot", frameRobot);
    
    this->get_parameter("base_increment_rate", baseIncrementRate);
    this->get_parameter("decay_factor", decayFactor);
    this->get_parameter("max_range", maxRange);
    this->get_parameter("fov_deg", fovDeg);
    this->get_parameter("relevance_threshold", relevanceThreshold);
    this->get_parameter("debug_mode", debugMode);
    this->get_parameter("raytracing_enabled", raytracingEnabled);
    this->get_parameter("prompt_topic", promptTopic);
    this->get_parameter("angular_confidence_sharpness", angularSharpness);
    this->get_parameter("radial_confidence_sharpness", radialSharpness);
    this->get_parameter("ignore_high_score_threshold", ignoreHighScoreThreshold);

    // TF
    tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // Subscriptions
    mapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        inputMapTopic, rclcpp::QoS(10),
        std::bind(&RelevanceMapNode::mapCallback, this, std::placeholders::_1));

    cameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        cameraInfoTopic,
        rclcpp::SensorDataQoS().best_effort(),
        std::bind(&RelevanceMapNode::cameraInfoCallback, this, std::placeholders::_1));

    graphNodeSub = this->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        inputGraphTopic,
        rclcpp::QoS(10),
        std::bind(&RelevanceMapNode::graphNodeCallback, this, std::placeholders::_1));

    promptSub = this->create_subscription<multimodal_query_msgs::msg::SemanticPrompt>(
        promptTopic, rclcpp::QoS(10),
        std::bind(&RelevanceMapNode::promptCallback, this, std::placeholders::_1));

    // Publisher
    graphNodeFilteredPub = this->create_publisher<graph_node_msgs::msg::GraphNodeArray>(
        outputGraphTopic, rclcpp::QoS(10));

    
    if (debugMode) {
        relevanceMapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "relevance_map", rclcpp::QoS(1).transient_local());
        RCLCPP_WARN(this->get_logger(), "Debug mode enabled – publishing /debug/relevance_map");
    }

    robot = std::make_shared<Robot>(this);

    paramCallbackHandle = this->add_on_set_parameters_callback(
    std::bind(&RelevanceMapNode::onParameterChange, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RelevanceMapNode initialized.");
}

void RelevanceMapNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    resizeRelevanceMap(msg);
}

void RelevanceMapNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camInfo = msg;
    fovDeg = computeFovFromCameraInfo();
}


void RelevanceMapNode::promptCallback(const multimodal_query_msgs::msg::SemanticPrompt::SharedPtr)
{
    resetRelevanceMap();
}

void RelevanceMapNode::resetRelevanceMap()
{
    if (!mapInitialized)
        return;

    relevanceMap.setTo(0.0f);
    RCLCPP_INFO(this->get_logger(), "Relevance map cleared due to new prompt.");
}

void RelevanceMapNode::resizeRelevanceMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    const int newHeight = msg->info.height;
    const int newWidth  = msg->info.width;
    const float newRes  = msg->info.resolution;

    if (!mapInitialized || relevanceMap.empty())
    {
        relevanceMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
        map = msg;
        mapInitialized = true;
        RCLCPP_INFO(this->get_logger(), "Initialized relevance map for the first time.");
        return;
    }

    const auto &oldInfo = map->info;
    const auto &oldO = oldInfo.origin.position;
    const auto &newO = msg->info.origin.position;

    // Convert both origins to map frame relative to the same TF
    geometry_msgs::msg::TransformStamped tf_map_to_base;
    try {
        tf_map_to_base = tfBuffer->lookupTransform(frameMap, frameRobot, tf2::TimePointZero);
    } catch (...) {
        tf_map_to_base = geometry_msgs::msg::TransformStamped();
    }

    // Compensate map movement using robot anchor
    const float anchorX = tf_map_to_base.transform.translation.x;
    const float anchorY = tf_map_to_base.transform.translation.y;

    // Compute effective delta in map pixels
    const float res = newRes;
    const int dx = static_cast<int>(std::round((oldO.x - newO.x + (anchorX - anchorX)) / res));
    const int dy = static_cast<int>(std::round((oldO.y - newO.y + (anchorY - anchorY)) / res));

    const bool sizeChanged   = (relevanceMap.rows != newHeight) || (relevanceMap.cols != newWidth);
    const bool originChanged = (dx != 0) || (dy != 0);

    if (!sizeChanged && !originChanged)
    {
        map = msg;
        return;
    }

    // Create new map with same size as updated occupancy grid
    cv::Mat newRelevance = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);

    // Compute overlap region (avoid copying out of bounds)
    const int src_x0 = std::max(0, -dx);
    const int src_y0 = std::max(0, -dy);
    const int dst_x0 = std::max(0,  dx);
    const int dst_y0 = std::max(0,  dy);

    int copyW = std::min(relevanceMap.cols - src_x0, newWidth  - dst_x0);
    int copyH = std::min(relevanceMap.rows - src_y0, newHeight - dst_y0);

    if (copyW > 0 && copyH > 0)
    {
        cv::Rect srcR(src_x0, src_y0, copyW, copyH);
        cv::Rect dstR(dst_x0, dst_y0, copyW, copyH);
        relevanceMap(srcR).copyTo(newRelevance(dstR));
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
            "No overlap after map origin/size change (dx=%d, dy=%d). Relevance map reset.",
            dx, dy);
    }

    relevanceMap = std::move(newRelevance);
    map = msg;

    RCLCPP_INFO(this->get_logger(),
        "Relevance map updated: (%dx%d) -> (%dx%d), origin shift dx=%d, dy=%d px",
        oldInfo.width, oldInfo.height, newWidth, newHeight, dx, dy);
}


float RelevanceMapNode::getYaw(const geometry_msgs::msg::Pose &pose) const
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

cv::Mat RelevanceMapNode::computeFovMask(const geometry_msgs::msg::Pose &pose)
{
    int w = map->info.width;
    int h = map->info.height;
    float res = map->info.resolution;

    cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);

    float robotX = (pose.position.x - map->info.origin.position.x) / res;
    float robotY = (pose.position.y - map->info.origin.position.y) / res;

    float yaw = getYaw(pose);

    float fov_deg = static_cast<float>(computeFovFromCameraInfo());
    float halfFov = (fov_deg * M_PI / 180.0f) * 0.5f;
    
    int maxSteps = static_cast<int>(maxRange / res);

    for (int i = 0; i < 200; i++) {
        float angle = yaw - halfFov + (float(i) / 199.0f) * (2.0f * halfFov);
        float dx = std::cos(angle);
        float dy = std::sin(angle);

        for (int step = 0; step < maxSteps; step++) {
            int px = static_cast<int>(robotX + dx * step);
            int py = static_cast<int>(robotY + dy * step);

            if (px < 0 || py < 0 || px >= w || py >= h)
                break;

            int idx = py * w + px;
            int8_t cell = map->data[idx];

            if (cell == -1) {
                // Unknown — allow if raytracing disabled, skip if enabled
                if (!raytracingEnabled)
                    mask.at<uint8_t>(py, px) = 1;
                continue;
            }

            if (raytracingEnabled) {
                // Stop ray once a wall is hit
                if (cell >= 50) {
                    mask.at<uint8_t>(py, px) = 1;  // mark the boundary cell
                    break;
                } else {
                    mask.at<uint8_t>(py, px) = 1;  // free space before wall
                }
            } else {
                // No raytracing → fill all free and occupied cells alike
                mask.at<uint8_t>(py, px) = 1;
            }
        }

    }

    return mask;
}

double RelevanceMapNode::computeDistanceGaussian(double dist, double sigma) const
{
    return std::exp(- (dist * dist) / (2.0 * sigma * sigma));
}

void RelevanceMapNode::integrateRelevance(const geometry_msgs::msg::Pose &pose)
{
    if (!mapInitialized)
        return;

    int w = map->info.width;
    int h = map->info.height;
    float res = map->info.resolution;

    // Apply decay to previous relevance values
    relevanceMap *= decayFactor;

    // Compute field of view mask (with optional raytracing)
    cv::Mat fovMask = computeFovMask(pose);

    // Robot position in map pixels
    float robotX = (pose.position.x - map->info.origin.position.x) / res;
    float robotY = (pose.position.y - map->info.origin.position.y) / res;
    float yaw = getYaw(pose);

    // Radial Gaussian width parameter
    double sigma = maxRange * radialSharpness;

    // Angular Gaussian width (controls how sharply confidence decays toward FOV edges)
    double angularSigma = (fovDeg * M_PI / 180.0) * angularSharpness;

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            if (fovMask.at<uint8_t>(y, x) == 0)
                continue;

            // Compute distance from robot
            float dx = static_cast<float>(x) - robotX;
            float dy = static_cast<float>(y) - robotY;
            double dist = std::sqrt(dx * dx + dy * dy) * res;

            // Radial Gaussian weighting (distance-based confidence)
            double radialWeight = std::exp(- (dist * dist) / (2.0 * sigma * sigma));

            // Angular Gaussian weighting (center-of-FOV confidence)
            float angle = std::atan2(dy, dx);
            float angleDiff = std::fabs(normalizeAngle(angle - yaw));
            double angularWeight = std::exp(-0.5 * std::pow(angleDiff / angularSigma, 2.0));

            // Combine both weights
            double combinedWeight = radialWeight * angularWeight;

            // Update relevance incrementally
            relevanceMap.at<float>(y, x) += static_cast<float>(baseIncrementRate * combinedWeight);
        }
    }

    publishRelevanceMap();
}

void RelevanceMapNode::graphNodeCallback(
    const graph_node_msgs::msg::GraphNodeArray::SharedPtr msg)
{
    if (!mapInitialized)
        return;

    auto pose = robot->getPose();
    if (!pose)
        return;

    integrateRelevance(pose->pose);

    graph_node_msgs::msg::GraphNodeArray out;
    out.header = msg->header;

    for (auto &node : msg->nodes)
    {
        // --- Skip and republish directly if above ignore threshold ---
        if (node.score >= ignoreHighScoreThreshold)
        {
            if (debugMode)
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "[RelevanceMapNode] Ignoring high-score node (%.2f ≥ %.2f) → Direct republish.",
                    node.score, ignoreHighScoreThreshold);

            out.nodes.push_back(node);
            continue;
        }

        float col = (node.position.x - map->info.origin.position.x) / map->info.resolution;
        float row = (node.position.y - map->info.origin.position.y) / map->info.resolution;

        if (row < 0 || col < 0 || row >= relevanceMap.rows || col >= relevanceMap.cols)
            continue;

        float rel = relevanceMap.at<float>((int)row, (int)col);
        node.relevance = rel;

        if (rel < relevanceThreshold)
            out.nodes.push_back(node);
    }

    graphNodeFilteredPub->publish(out);
}

void RelevanceMapNode::publishRelevanceMap()
{
    if (!debugMode || !mapInitialized || !map)
        return;

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = map->header.frame_id;
    msg.info = map->info;

    msg.data.resize(relevanceMap.rows * relevanceMap.cols);

    for (int y = 0; y < relevanceMap.rows; ++y) {
        for (int x = 0; x < relevanceMap.cols; ++x) {
            float val = relevanceMap.at<float>(y, x);
            // Scale to [0,100] for RViz display
            int idx = y * relevanceMap.cols + x;
            msg.data[idx] = static_cast<int8_t>(std::clamp(val * 100.0f, 0.0f, 100.0f));
        }
    }

    relevanceMapPub->publish(msg);
}

rcl_interfaces::msg::SetParametersResult RelevanceMapNode::onParameterChange(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Dynamic parameters updated";

    for (const auto &p : params) {
        const auto &name = p.get_name();
        if (name == "base_increment_rate") {
            baseIncrementRate = p.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated base_increment_rate: %.3f", baseIncrementRate);
        } else if (name == "decay_factor") {
            decayFactor = p.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated decay_factor: %.3f", decayFactor);
        } else if (name == "max_range") {
            maxRange = p.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated max_range: %.3f", maxRange);
        } else if (name == "fov_deg") {
            fovDeg = p.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated fov_deg: %.3f", fovDeg);
        } else if (name == "relevance_threshold") {
            relevanceThreshold = p.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated relevance_threshold: %.3f", relevanceThreshold);
        } else if (name == "debug_mode") {
            debugMode = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "Updated debug_mode: %s", debugMode ? "true" : "false");
        } else if (name == "raytracing_enabled") {
            raytracingEnabled = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "Updated raytracing_enabled: %s", raytracingEnabled ? "true" : "false");
        } else if (name == "angular_confidence_sharpness") {
            angularSharpness = p.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated angular_confidence_sharpness: %.3f", angularSharpness);
        } else if (name == "radial_confidence_sharpness") {
            radialSharpness = p.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated radial_confidence_sharpness: %.3f", radialSharpness);
        } else if (name == "ignore_high_score_threshold") {
            ignoreHighScoreThreshold = p.as_double();
            RCLCPP_INFO(this->get_logger(),
                        "Updated ignore_high_score_threshold: %.3f", ignoreHighScoreThreshold);
        }
    }

    return result;
}

double RelevanceMapNode::computeFovFromCameraInfo() const
{
    if (!camInfo)
        return fovDeg; // fallback to parameter if camera info not available

    double fx = camInfo->k[0];
    double width = static_cast<double>(camInfo->width);

    if (fx <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "CameraInfo fx invalid (%.3f), using fallback FOV %.1f deg.", fx, fovDeg);
        return fovDeg;
    }

    double fov_rad = 2.0 * std::atan(width / (2.0 * fx));
    double fov_deg = fov_rad * 180.0 / M_PI;
    return fov_deg;
}
