#include "semantic_value_map.hpp"

static inline float pack_rgb_u32_to_float(uint32_t rgb_u32) {
  union { uint32_t u; float f; } c; c.u = rgb_u32; return c.f;
}


bool hasRobotMoved(const geometry_msgs::msg::Pose& last,
                        const geometry_msgs::msg::Pose& current,
                        double moveThresh)
{
    const double dx = current.position.x - last.position.x;
    const double dy = current.position.y - last.position.y;
    const double dist = std::hypot(dx, dy);
    return dist > moveThresh;
}

inline float gaussianDistanceWeight(float dx, float dy, float sigma_dist)
{
    float d2 = dx*dx + dy*dy;
    return std::exp(-0.5f * d2 / (sigma_dist * sigma_dist));
}

SemanticValueMap::SemanticValueMap(rclcpp_lifecycle::LifecycleNode* node)
: node(node)
{

}

SemanticValueMap::~SemanticValueMap() 
{

}

bool SemanticValueMap::on_configure()
{
    RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Configuring...");

    node->declare_parameter<std::string>("semantic_map.clear_service_name", "/clear_value_map");

    node->get_parameter("semantic_map.clear_service_name", clearValueMapServiceName);

    // --- Existing parameter declarations below ---
    node->declare_parameter<float>("semantic_map.confidence_sharpness", 2.0);
    node->declare_parameter<double>("semantic_map.decay_factor", 0.99);
    node->declare_parameter<float>("semantic_map.max_range", 10.0f);
    node->declare_parameter<std::string>("semantic_map.map_topic", "/map");
    node->declare_parameter<std::string>("semantic_map.camera_info_topic", "/camera_info");
    node->declare_parameter<bool>("semantic_map.visualize_confidence_map", false);
    node->declare_parameter<float>("semantic_map.update_gain", 0.3f);
    node->declare_parameter<bool>("semantic_map.use_camera_info_fov", true);
    node->declare_parameter<double>("semantic_map.fov", 30.0);
    node->declare_parameter<double>("semantic_map.move_threshold", 0.05);
    node->declare_parameter<float>("semantic_map.distance_sigma", 3.0f);

    // --- Load parameters (existing code) ---
    node->get_parameter("semantic_map.confidence_sharpness", confidenceSharpness);
    node->get_parameter("semantic_map.decay_factor", decayFactor);
    node->get_parameter("semantic_map.max_range", maxRange);
    node->get_parameter("semantic_map.map_topic", mapTopic);
    node->get_parameter("semantic_map.camera_info_topic", cameraInfoTopic);
    node->get_parameter("semantic_map.visualize_confidence_map", visualizeConfidenceMap);
    node->get_parameter("semantic_map.update_gain", updateGain);
    node->get_parameter("semantic_map.use_camera_info_fov", useCameraInfoFov);
    node->get_parameter("semantic_map.fov", fovDeg);
    node->get_parameter("semantic_map.move_threshold", moveThreshold);
    node->get_parameter("semantic_map.distance_sigma", distanceSigma);

    // --- Service creation ---
    clearValueMapService = node->create_service<std_srvs::srv::Empty>(
        clearValueMapServiceName,
        std::bind(&SemanticValueMap::handleClearValueMap, this,
                  std::placeholders::_1, std::placeholders::_2));

    parameterCallbackHandle = node->add_on_set_parameters_callback(
        std::bind(&SemanticValueMap::onParameterChange, this, std::placeholders::_1));


    RCLCPP_INFO(node->get_logger(), "Created clear value map service: %s",
                clearValueMapServiceName.c_str());

    // --- Existing subscriptions and publishers (unchanged) ---
    mapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        mapTopic, 10, std::bind(&SemanticValueMap::mapCallback, this, std::placeholders::_1));

    cameraInfoSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        cameraInfoTopic, 10, std::bind(&SemanticValueMap::cameraInfoCallback, this, std::placeholders::_1));

    valueMapInfernoPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("value_map", rclcpp::QoS(10));
    valueMapRawPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("value_map_raw", rclcpp::QoS(10));

    return true;
}

rcl_interfaces::msg::SetParametersResult SemanticValueMap::onParameterChange(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated";

    for (const auto &param : params)
    {
        if (param.get_name() == "semantic_map.confidence_sharpness") {
            confidenceSharpness = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated confidence_sharpness to %.2f", confidenceSharpness);
        } else if (param.get_name() == "semantic_map.decay_factor") {
            decayFactor = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated decay_factor to %.2f", decayFactor);
        } else if (param.get_name() == "semantic_map.max_range") {
            maxRange = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated max_range to %.2f", maxRange);
        } else if (param.get_name() == "semantic_map.map_topic") {
            mapTopic = param.as_string();
            RCLCPP_INFO(node->get_logger(), "Updated map_topic to %s", mapTopic.c_str());
        } else if (param.get_name() == "semantic_map.camera_info_topic") {
            cameraInfoTopic = param.as_string();
            RCLCPP_INFO(node->get_logger(), "Updated camera_info_topic to %s", cameraInfoTopic.c_str());
        } else if (param.get_name() == "semantic_map.visualize_confidence_map") {
            visualizeConfidenceMap = param.as_bool();
            RCLCPP_INFO(node->get_logger(), "Updated visualize_confidence_map to %s", visualizeConfidenceMap ? "true" : "false");
        } else if (param.get_name() == "semantic_map.update_gain") {
            updateGain = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated update_gain to %.2f", updateGain);
        } else if (param.get_name() == "semantic_map.use_camera_info_fov") {
            useCameraInfoFov = param.as_bool();
            RCLCPP_INFO(node->get_logger(), "Updated use_camera_info_fov to %s", useCameraInfoFov ? "true" : "false");
        } else if (param.get_name() == "semantic_map.fov") {
            fovDeg = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated fov to %.2f", fovDeg);
        } else if (param.get_name() == "semantic_map.move_threshold") {
            moveThreshold = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated move_threshold to %.2f", moveThreshold);
        } else if (param.get_name() == "semantic_map.distance_sigma") {
            distanceSigma = param.as_double();
            RCLCPP_INFO(node->get_logger(), "Updated distance_sigma to %.2f", distanceSigma);
        }
    }

    return result;
}

bool SemanticValueMap::on_activate() 
{
  RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Activating...");

  valueMapInfernoPub->on_activate();
  
  valueMapRawPub->on_activate();

  return true;
}

bool SemanticValueMap::on_deactivate() 
{
  RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Deactivating...");

  valueMapInfernoPub->on_deactivate();

  valueMapRawPub->on_deactivate();

  return true;
}

bool SemanticValueMap::on_cleanup() 
{
  RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Cleaning up...");

  valueMapInfernoPub.reset();

  valueMapRawPub.reset();

  return true;
}

bool SemanticValueMap::on_shutdown() 
{
  RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Shutting down...");
  on_cleanup();
  return true;
}

void SemanticValueMap::updateSemanticMap(const SemanticScore& semScore,
                                         const geometry_msgs::msg::PoseStamped& pose)
{
    if (!map)
    {
        RCLCPP_WARN(node->get_logger(), "No occupancy grid map received yet.");
        return;
    }

    if (useCameraInfoFov)
        fovDeg = getHorizontalFOV(camInfo);

    // --- Step 0: Decide if robot moved in XY --- //
    bool moved = true;
    if (lastPoseValid)
        moved = hasRobotMoved(lastPose, pose.pose, moveThreshold);

    if (!moved)
    {
        RCLCPP_DEBUG(node->get_logger(),
                     "Robot only rotated or barely moved (Δ < %.2fm) → skipping decay.",
                     moveThreshold);
    }

    // --- Step 1: Apply decay only if translation occurred --- //
    if (moved)
    {
        valueMap *= decayFactor;
        confidenceMap *= decayFactor;
    }

    // Save pose for next iteration
    lastPose = pose.pose;
    lastPoseValid = true;

    // --- Continue with your existing logic --- //
    cv::Mat fovMask = generateTopdownConeMask(pose.pose, *map, fovDeg, maxRange);
    cv::Mat confidence = computeConfidenceMap(pose.pose, *map, fovDeg, fovMask);

    int width = map->info.width;
    int height = map->info.height;
    float newValue = semScore.getScore();
    float resolution = map->info.resolution;
    distanceSigmaPx = distanceSigma / resolution;

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col)
        {
            if (fovMask.at<uint8_t>(row, col) == 0)
                continue;

            float prevValue = valueMap.at<float>(row, col);
            float prevConf  = confidenceMap.at<float>(row, col);
            float newConf   = confidence.at<float>(row, col);

            if ((prevConf + newConf) == 0.0f)
                continue;

            float effConf = newConf * updateGain;
            // Here newValue = cosine similarity from SEEM
            // Normalize to [0,1] if desired:
            float reward = (newValue + 1.0f) * 0.5f;

            // incremental reward accumulation
            float fusedValue = prevValue + effConf * reward;
            float fusedConf  = std::max(prevConf * static_cast<float>(decayFactor), newConf);

            valueMap.at<float>(row, col) = fusedValue;
            confidenceMap.at<float>(row, col) = fusedConf;
        }
    }

    publishValueMapRaw();
    publishValueMapInferno();
}


void SemanticValueMap::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
  resizeMapPreservingValues(msg);
}

void SemanticValueMap::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) 
{
  camInfo = msg;
}

void SemanticValueMap::resizeMapPreservingValues(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  const int   newHeight = msg->info.height;
  const int   newWidth  = msg->info.width;
  const float newRes    = msg->info.resolution;

  if (!mapInitialized || valueMap.empty() || confidenceMap.empty())
  {
    valueMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
    confidenceMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
    map = msg;
    mapInitialized = true;
    RCLCPP_INFO(node->get_logger(), "Initialized value/confidence maps.");
    return;
  }

  const auto &oldInfo = map->info;
  const auto &oldO = oldInfo.origin.position;
  const auto &newO = msg->info.origin.position;

  // If resolution changed, clear to avoid resampling artifacts
  if (std::fabs(oldInfo.resolution - newRes) > 1e-9) {
    RCLCPP_WARN(node->get_logger(), "Map resolution changed (%.6f -> %.6f). Clearing maps.",
                oldInfo.resolution, newRes);
    valueMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
    confidenceMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
    map = msg;
    return;
  }

  // Pixel shift due to origin change: positive means move content to +x/+y in new grid.
  const float res = newRes;
  const int dx = static_cast<int>(std::round((oldO.x - newO.x) / res));
  const int dy = static_cast<int>(std::round((oldO.y - newO.y) / res));

  const bool size_changed   = (valueMap.rows != newHeight) || (valueMap.cols != newWidth);
  const bool origin_changed = (dx != 0) || (dy != 0);

  if (!size_changed && !origin_changed) {
    map = msg;  // nothing to do
    return;
  }

  cv::Mat newValueMap      = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
  cv::Mat newConfidenceMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);

  // Compute overlapped rectangles with shift applied
  const int src_x0 = std::max(0, -dx);
  const int src_y0 = std::max(0, -dy);
  const int dst_x0 = std::max(0,  dx);
  const int dst_y0 = std::max(0,  dy);

  int copy_w = std::min(valueMap.cols - src_x0, newWidth  - dst_x0);
  int copy_h = std::min(valueMap.rows - src_y0, newHeight - dst_y0);

  if (copy_w > 0 && copy_h > 0) {
    cv::Rect srcR(src_x0, src_y0, copy_w, copy_h);
    cv::Rect dstR(dst_x0, dst_y0, copy_w, copy_h);
    valueMap(srcR).copyTo(newValueMap(dstR));
    confidenceMap(srcR).copyTo(newConfidenceMap(dstR));
  } else {
    RCLCPP_WARN(node->get_logger(), "No overlap after origin/size change (dx=%d, dy=%d). Maps cleared.", dx, dy);
  }

  valueMap = std::move(newValueMap);
  confidenceMap = std::move(newConfidenceMap);
  map = msg;

  RCLCPP_INFO(node->get_logger(),
              "Map updated: (%dx%d)->(%dx%d), origin shift: dx=%d, dy=%d px",
              oldInfo.width, oldInfo.height, newWidth, newHeight, dx, dy);
}

double SemanticValueMap::getHorizontalFOV(const sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo)
{
  if(!cameraInfo)
  {
    RCLCPP_WARN(node->get_logger(), "Camera Info not set. Did /camera_info publishished yet?");
    RCLCPP_INFO(node->get_logger(), "Using FOV Fallback default %f", fovDeg);
    return fovDeg;
  }
  double width = cameraInfo->width;
  double fx = cameraInfo->k[0];
  double fov = 2 * atan2(width / 2, fx ) * 180 / M_PI;

  return fov;
}

void SemanticValueMap::publishValueMapInferno() 
{
  if (!map) return;

  const auto& info = map->info;
  const auto& origin = info.origin;
  const int width = info.width;
  const int height = info.height;
  const float resolution = info.resolution;

  // Collect valid points first
  std::vector<std::array<float,4>> pts;  // x,y,z,rgb
  pts.reserve(width * height / 8);       // heuristic

  for (int row = 0; row < height; ++row) {
    const float y = origin.position.y + row * resolution;
    const float x0 = origin.position.x; // for speed
    for (int col = 0; col < width; ++col) {
      const float v = valueMap.at<float>(row, col);
      if (v <= 0.0f) continue;

      const float x = x0 + col * resolution;
      if (v > maxSemanticScore) maxSemanticScore = v;
      auto [r, g, b] = valueToInfernoRGB(v, 0.0f, maxSemanticScore);
      uint32_t rgb_u32 = (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
      pts.push_back({x, y, 0.0f, pack_rgb_u32_to_float(rgb_u32)});
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = node->get_clock()->now();           // << stamp with "now" to match TF
  cloud.header.frame_id = map->header.frame_id;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> irgb(cloud, "rgb");

  for (const auto& p : pts) {
    *ix = p[0]; ++ix;
    *iy = p[1]; ++iy;
    *iz = p[2]; ++iz;
    *irgb = p[3]; ++irgb;
  }

  valueMapInfernoPub->publish(cloud);
}

void SemanticValueMap::publishValueMapRaw() 
{
  if (!map) return;

  const auto& info = map->info;
  const auto& origin = info.origin;
  const int width = info.width;
  const int height = info.height;
  const float resolution = info.resolution;

  std::vector<std::array<float,4>> pts;  // x,y,z,intensity
  pts.reserve(width * height / 8);

  for (int row = 0; row < height; ++row) {
    const float y = origin.position.y + row * resolution;
    const float x0 = origin.position.x;
    for (int col = 0; col < width; ++col) {
      const float v = valueMap.at<float>(row, col);
      if (v <= 0.0f) continue;
      const float x = x0 + col * resolution;
      pts.push_back({x, y, 0.0f, v});
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = node->get_clock()->now();           // << up-to-date stamp
  cloud.header.frame_id = map->header.frame_id;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> ii(cloud, "intensity");

  for (const auto& p : pts) {
    *ix = p[0]; ++ix;
    *iy = p[1]; ++iy;
    *iz = p[2]; ++iz;
    *ii = p[3]; ++ii;
  }

  valueMapRawPub->publish(cloud);
}

cv::Mat SemanticValueMap::generateTopdownConeMask(
    const geometry_msgs::msg::Pose& pose,
    const nav_msgs::msg::OccupancyGrid& grid,
    float fovDeg,
    float maxRange)
{
  int width = grid.info.width;
  int height = grid.info.height;
  float resolution = grid.info.resolution;
  const auto& origin = grid.info.origin;

  cv::Mat fovMask = cv::Mat::zeros(height, width, CV_8UC1);

  float robotX = (pose.position.x - origin.position.x) / resolution;
  float robotY = (pose.position.y - origin.position.y) / resolution;

  // --- Compute robot yaw --- //
  float yaw = getYawAngle(pose);

  float fovRad = fovDeg * M_PI / 180.0f;
  float halfFov = fovRad / 2.0f;
  int maxRangePx = static_cast<int>(maxRange / resolution);

  int numRays = 200;
  for (int i = 0; i < numRays; ++i)
  {
    float angle = yaw - halfFov + (static_cast<float>(i) / (numRays - 1)) * fovRad;
    float dx = std::cos(angle);
    float dy = std::sin(angle);

    for (int step = 0; step < maxRangePx; ++step)
    {
      int px = static_cast<int>(robotX + dx * step);
      int py = static_cast<int>(robotY + dy * step);

      if (px < 0 || px >= width || py < 0 || py >= height)
        break;

      int idx = py * width + px;
      int8_t cell = grid.data[idx];

      if (cell == -1)
        continue;  // unknown
      else if (cell >= 50)
      {
        fovMask.at<uint8_t>(py, px) = 1;  // mark occupied cell
        break;  // stop ray at occupied cell
      }
      else
      {
        fovMask.at<uint8_t>(py, px) = 1;  // mark free cell
      }
    }
  }

  return fovMask;
}

cv::Mat SemanticValueMap::computeConfidenceMap(
    const geometry_msgs::msg::Pose& pose,
    const nav_msgs::msg::OccupancyGrid& grid,
    float cameraFovDeg,
    const cv::Mat& fovMask)
{
    int height = fovMask.rows;
    int width = fovMask.cols;

    cv::Mat confidenceMap = cv::Mat::zeros(height, width, CV_32FC1);

    float resolution = grid.info.resolution;
    const auto& origin = grid.info.origin;

    // Robot position in map pixels
    float robotX = (pose.position.x - origin.position.x) / resolution;
    float robotY = (pose.position.y - origin.position.y) / resolution;

    float yaw = getYawAngle(pose);
    float halfFov = (cameraFovDeg * M_PI / 180.0f) / 2.0f;

    float sigma = (halfFov) / confidenceSharpness;  // smaller sharpness → wider bell

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {

            if (fovMask.at<uint8_t>(row, col) == 0)
                continue;

            float dx = static_cast<float>(col) - robotX;
            float dy = static_cast<float>(row) - robotY;

            float angle = std::atan2(dy, dx);
            float angleDiff = normalizeAngle(angle - yaw);

            // --- Skip cells outside camera FOV --- //
            if (std::abs(angleDiff) > halfFov)
                continue;

            // --- ANGLE WEIGHT (existing) --- //
            float angleW = std::exp(-0.5f * std::pow(angleDiff / sigma, 2.0f));

            // --- DISTANCE WEIGHT (new) --- //
            float distW = gaussianDistanceWeight(dx, dy, distanceSigmaPx);

            // --- Combined final weight --- //
            float weight = angleW * distW;

            confidenceMap.at<float>(row, col) = weight;
        }
    }

    if (visualizeConfidenceMap) {
        cv::Mat vis;
        confidenceMap.convertTo(vis, CV_8UC1, 255.0);
        cv::flip(vis, vis, 0);
        cv::resize(vis, vis, cv::Size(), 3.0, 3.0, cv::INTER_NEAREST);
        cv::imshow("Confidence Map", vis);
        cv::waitKey(1);
        confidenceWindowOpen = true;
    } else if (confidenceWindowOpen) {
        cv::destroyWindow("Confidence Map");
        confidenceWindowOpen = false;
    }
    return confidenceMap;
}

float SemanticValueMap::getYawAngle(const geometry_msgs::msg::Pose& pose) const
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
    
return tf2::getYaw(q) + M_PI_2;
}

float SemanticValueMap::normalizeAngle(float angle) const
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void SemanticValueMap::handleClearValueMap(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    clearValueMap();
}

void SemanticValueMap::clearValueMap()
{
    if (!mapInitialized || valueMap.empty() || confidenceMap.empty()) {
        RCLCPP_WARN(node->get_logger(), "Cannot clear value map: not initialized yet.");
        return;
    }

    valueMap.setTo(0.0f);
    confidenceMap.setTo(0.0f);
    maxSemanticScore = 0.0f;

    RCLCPP_INFO(node->get_logger(), "SemanticValueMap: Cleared all semantic and confidence values.");

    // Optionally republish cleared maps as empty PointClouds
    publishValueMapRaw();
    publishValueMapInferno();
}