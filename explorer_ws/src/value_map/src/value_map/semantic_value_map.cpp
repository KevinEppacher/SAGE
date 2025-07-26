#include "semantic_value_map.hpp"

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

  mapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&SemanticValueMap::mapCallback, this, std::placeholders::_1));

  cameraInfoSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", 10, std::bind(&SemanticValueMap::cameraInfoCallback, this, std::placeholders::_1));

  valueMapInfernoPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("value_map", rclcpp::QoS(10));
  
  valueMapRawPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("value_map_raw", rclcpp::QoS(10));


  return true;
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

void SemanticValueMap::updateSemanticMap(const SemanticScore& semScore, const geometry_msgs::msg::PoseStamped& pose)
{
  if (!map)
  {
    RCLCPP_WARN(node->get_logger(), "No occupancy grid map received yet.");
    return;
  }

  double fovDeg = getHorizontalFOV(camInfo);
  RCLCPP_INFO(node->get_logger(), "Using FOV angle: %.2f", fovDeg);

  // --- Step 1: Decay previous values ---
  double decayFactor = 0.95;
  valueMap *= decayFactor;
  confidenceMap *= decayFactor;
  RCLCPP_INFO(node->get_logger(), "Applied decay to value and confidence maps.");

  // --- Step 2: Generate FOV cone mask ---
  cv::Mat fovMask = generateTopdownConeMask(pose.pose, *map, fovDeg, 10.0);
  RCLCPP_INFO(node->get_logger(), "FOV mask generated. Non-zero count: %d", cv::countNonZero(fovMask));

  // --- Step 3: Compute confidence values ---
  cv::Mat confidence = computeConfidenceMap(pose.pose, *map, fovDeg, fovMask);
  RCLCPP_INFO(node->get_logger(), "Confidence map computed.");

  int width = map->info.width;
  int height = map->info.height;

  float newValue = semScore.getScore();
  RCLCPP_INFO(node->get_logger(), "New semantic score: %.2f", newValue);

  int updates = 0;
  for (int row = 0; row < height; ++row)
  {
    for (int col = 0; col < width; ++col)
    {
      if (fovMask.at<uint8_t>(row, col) == 0)
        continue;

      float prevValue = valueMap.at<float>(row, col);
      float prevConf = confidenceMap.at<float>(row, col);
      float newConf = confidence.at<float>(row, col);

      if ((prevConf + newConf) == 0.0f)
        continue;

      float fusedValue = (prevValue * prevConf + newValue * newConf) / (prevConf + newConf);
      float fusedConf = (prevConf * prevConf + newConf * newConf) / (prevConf + newConf);

      valueMap.at<float>(row, col) = fusedValue;
      confidenceMap.at<float>(row, col) = fusedConf;

      ++updates;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Updated %d cells in value map.", updates);
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
  const int newHeight = msg->info.height;
  const int newWidth = msg->info.width;

  if (!mapInitialized || valueMap.empty() || confidenceMap.empty()) 
  {
    // First-time initialization
    valueMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
    confidenceMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
    map = msg;
    mapInitialized = true;

    RCLCPP_INFO(node->get_logger(), "Initialized value/confidence maps.");
    return;
  }

  if (valueMap.rows == newHeight && valueMap.cols == newWidth) 
  {
    // No resizing necessary
    map = msg;
    return;
  }

  RCLCPP_INFO(node->get_logger(), "Resizing value map from (%d, %d) to (%d, %d)",
              valueMap.rows, valueMap.cols, newHeight, newWidth);

  // Create new maps with updated size
  cv::Mat newValueMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);
  cv::Mat newConfidenceMap = cv::Mat::zeros(newHeight, newWidth, CV_32FC1);

  // Determine common overlapping region
  int minRows = std::min(valueMap.rows, newHeight);
  int minCols = std::min(valueMap.cols, newWidth);

  // Copy overlapping region into the new maps
  valueMap(cv::Rect(0, 0, minCols, minRows))
      .copyTo(newValueMap(cv::Rect(0, 0, minCols, minRows)));
  confidenceMap(cv::Rect(0, 0, minCols, minRows))
      .copyTo(newConfidenceMap(cv::Rect(0, 0, minCols, minRows)));

  // Replace old maps
  valueMap = newValueMap;
  confidenceMap = newConfidenceMap;
  map = msg;
}

double SemanticValueMap::getHorizontalFOV(const sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo)
{
  if(!cameraInfo)
  {
    RCLCPP_WARN(node->get_logger(), "Camera Info not set. Did /camera_info publishished yet?");
    RCLCPP_INFO(node->get_logger(), "Using FOV Fallback angle 45°.");
    return 45.0;
  }
  double width = cameraInfo->width;
  double fx = cameraInfo->k[0];
  double fov = 2 * atan2(width / 2, fx ) * 180 / M_PI;

  return fov;
}

void SemanticValueMap::publish()
{
  publishValueMapInferno();
  publishValueMapRaw();
}

void SemanticValueMap::publishValueMapInferno()
{
  if (!map) return;

  const auto& info = map->info;
  const auto& origin = info.origin;
  int width = info.width;
  int height = info.height;
  float resolution = info.resolution;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = map->header.stamp;
  cloud.header.frame_id = map->header.frame_id;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
  );
  modifier.resize(width * height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud, "rgb");

  for (int row = 0; row < height; ++row)
  {
    for (int col = 0; col < width; ++col)
    {
      float value = valueMap.at<float>(row, col);
      if (value > 0.0f)
      {
        float x = origin.position.x + col * resolution;
        float y = origin.position.y + row * resolution;
        float z = 0.0f;

        if (value > maxSemanticScore)
        {
          maxSemanticScore = value;
        }

        auto [r, g, b] = valueToInfernoRGB(value, 0.0f, maxSemanticScore);
        uint32_t rgb_uint = (r << 16) | (g << 8) | b;
        float rgb = *reinterpret_cast<float*>(&rgb_uint);

        *iter_x = x; ++iter_x;
        *iter_y = y; ++iter_y;
        *iter_z = z; ++iter_z;
        *iter_rgb = rgb; ++iter_rgb;
      }
    }
  }

  valueMapInfernoPub->publish(cloud);
}

void SemanticValueMap::publishValueMapRaw()
{
  if (!map) return;

  const auto& info = map->info;
  const auto& origin = info.origin;
  int width = info.width;
  int height = info.height;
  float resolution = info.resolution;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = map->header.stamp;
  cloud.header.frame_id = map->header.frame_id;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
  );
  modifier.resize(width * height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  for (int row = 0; row < height; ++row)
  {
    for (int col = 0; col < width; ++col)
    {
      float value = valueMap.at<float>(row, col);
      if (value > 0.0f)
      {
        float x = origin.position.x + col * resolution;
        float y = origin.position.y + row * resolution;
        float z = 0.0f;

        *iter_x = x; ++iter_x;
        *iter_y = y; ++iter_y;
        *iter_z = z; ++iter_z;
        *iter_intensity = value; ++iter_intensity;
      }
    }
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
  const auto& q = pose.orientation;
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

  for (int row = 0; row < height; ++row)
  {
    for (int col = 0; col < width; ++col)
    {
      if (fovMask.at<uint8_t>(row, col) == 0)
        continue;

      float dx = static_cast<float>(col) - robotX;
      float dy = static_cast<float>(row) - robotY;
      float angle = std::atan2(dy, dx);
      float angleDiff = normalizeAngle(angle - yaw);
      float scaledAngle = angleDiff / halfFov * (M_PI / 2.0f);

      if (std::abs(angleDiff) <= halfFov)
      {
        float weight = std::cos(scaledAngle) * std::cos(scaledAngle);
        confidenceMap.at<float>(row, col) = weight;
      }
    }
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
