#include "cloud_cluster.hpp"

CloudCluster::CloudCluster() : Node("cluster_node") 
{
    RCLCPP_INFO(this->get_logger(), "CloudCluster Node started");

    getParameters();

    paramCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&CloudCluster::onParameterChange, this, std::placeholders::_1)
    );

    // Define subscribers
    semanticPointcloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        semanticPointcloudTopic, 10,
        std::bind(&CloudCluster::semanticCloudCallback, this, std::placeholders::_1)
    );

    // Define publishers
    markerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "bounding_boxes", 10
    );
    

    // Define timers
    auto interval = std::chrono::duration<double>(1.0 / publishFrequency);
    RCLCPP_INFO(this->get_logger(), "Publishing with a time interval of %.3f seconds", interval.count());
    timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(interval),
        std::bind(&CloudCluster::timerCallback, this)
    );

    // Initialize the point cloud
    semanticPointcloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    graphNodes = std::make_shared<GraphNodeCollection>(this);

}

CloudCluster::~CloudCluster() 
{
    RCLCPP_INFO(this->get_logger(), "CloudCluster Node stopped");
}

void CloudCluster::getParameters()
{
    this->declare_parameter("publish_frequency", 2.0);
    this->declare_parameter("cluster_tolerance", 0.15);
    this->declare_parameter("min_cluster_size", 50);
    this->declare_parameter("max_cluster_size", 25000);
    this->declare_parameter("voxel_leaf_size", 0.03);
    this->declare_parameter("semantic_pointcloud_topic", "/openfusion_ros/semantic_pointcloud_xyzi");
    this->declare_parameter("target_frame", "map");
    this->declare_parameter("publish_bounding_boxes", true);

    this->get_parameter("publish_frequency", publishFrequency);
    this->get_parameter("cluster_tolerance", clusterTolerance);
    this->get_parameter("min_cluster_size", minClusterSize);
    this->get_parameter("max_cluster_size", maxClusterSize);
    this->get_parameter("voxel_leaf_size", voxelLeafSize);    
    this->get_parameter("semantic_pointcloud_topic", semanticPointcloudTopic);
    this->get_parameter("target_frame", targetFrame);
    this->get_parameter("publish_bounding_boxes", publishBoundingBoxesEnabled);

    // TF buffer + listener
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
    for (const auto &name : this->list_parameters({}, 10).names)
    {
        auto param = this->get_parameter(name);
        switch (param.get_type())
        {
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                RCLCPP_INFO(this->get_logger(), " - %s: %f", name.c_str(), param.as_double());
                break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                RCLCPP_INFO(this->get_logger(), " - %s: %ld", name.c_str(), param.as_int());
                break;
            case rclcpp::ParameterType::PARAMETER_STRING:
                RCLCPP_INFO(this->get_logger(), " - %s: %s", name.c_str(), param.as_string().c_str());
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
                RCLCPP_INFO(this->get_logger(), " - %s: %s", name.c_str(), param.as_bool() ? "true" : "false");
                break;
            default:
                RCLCPP_INFO(this->get_logger(), " - %s: [unsupported type]", name.c_str());
                break;
        }
    }
}

void CloudCluster::semanticCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // transform to target frame
    sensor_msgs::msg::PointCloud2 cloud_tf;
    try {
        auto T = tfBuffer->lookupTransform(targetFrame, msg->header.frame_id, msg->header.stamp,
                                           rclcpp::Duration::from_seconds(0.1));
        tf2::doTransform(*msg, cloud_tf, T);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "TF %s->%s failed: %s", msg->header.frame_id.c_str(), targetFrame.c_str(), ex.what());
        return;
    }

    // verify instance field
    bool has_instance = false;
    for (const auto& f : cloud_tf.fields) if (f.name == "instance") { has_instance = true; break; }
    if (!has_instance) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "PointCloud2 has no 'instance' field; skipping frame");
        return;
    }

    // clear map
    cloudsByInstance.clear();

    // iterators
    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_tf, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud_tf, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud_tf, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(cloud_tf, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> it_inst(cloud_tf, "instance");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_inst) {
        uint16_t inst = *it_inst;
        if (inst == 0) continue; // background

        pcl::PointXYZI p;
        p.x = *it_x; p.y = *it_y; p.z = *it_z; p.intensity = *it_i;

        auto& pc = cloudsByInstance[inst];
        if (!pc) pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pc->points.push_back(p);
    }

    // optional: keep a merged copy if other code still reads semanticPointcloud
    semanticPointcloud->clear();
    for (auto& kv : cloudsByInstance) *semanticPointcloud += *kv.second;
}

rcl_interfaces::msg::SetParametersResult CloudCluster::onParameterChange(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated";

    for (const auto &param : params)
    {
        if (param.get_name() == "publish_frequency") {
            publishFrequency = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated publish_frequency to %.2f", publishFrequency);
        } else if (param.get_name() == "cluster_tolerance") {
            clusterTolerance = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated cluster_tolerance to %.2f", clusterTolerance);
        } else if (param.get_name() == "min_cluster_size") {
            minClusterSize = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Updated min_cluster_size to %d", minClusterSize);
        } else if (param.get_name() == "max_cluster_size") {
            maxClusterSize = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Updated max_cluster_size to %d", maxClusterSize);
        } else if (param.get_name() == "voxel_leaf_size") {
            voxelLeafSize = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated voxel_leaf_size to %.3f", voxelLeafSize);
        } else if (param.get_name() == "publish_bounding_boxes") {
            publishBoundingBoxesEnabled = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Updated publish_bounding_boxes to %s", 
                        publishBoundingBoxesEnabled ? "true" : "false");
        }
    }

    return result;
}

geometry_msgs::msg::Point CloudCluster::getCentroid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    geometry_msgs::msg::Point centroid;
    if (cloud->empty()) return centroid;

    double x = 0, y = 0, z = 0;
    for (const auto& pt : cloud->points) 
    {
        x += pt.x;
        y += pt.y;
        z += pt.z;
    }

    size_t n = cloud->points.size();
    centroid.x = x / n;
    centroid.y = y / n;
    centroid.z = z / n;

    return centroid;
}

// double CloudCluster::getScoreCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
// {
//     if (cluster->empty()) return 0.0;

//     std::vector<float> red_values;
//     for (const auto& pt : cluster->points)
//     {
//         float r = static_cast<float>(pt.intensity) / 255.0f;  // Normalize to 0–1
//         if (std::isfinite(r)) red_values.push_back(r);
//     }

//     if (red_values.empty()) return 0.0;

//     std::sort(red_values.begin(), red_values.end());

//     // Use e.g. the 75th percentile → robust against a few bright pixels
//     size_t index = static_cast<size_t>(0.75 * red_values.size());
//     float percentile_score = red_values[std::min(index, red_values.size() - 1)];

//     // Damp with cluster size (logarithmic growth)
//     double size_factor = std::log(static_cast<double>(cluster->size()) + 1.0);  // +1 to avoid log(0)
//     double score = percentile_score * size_factor;

//     return score;
// }

double CloudCluster::getScoreCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{
    if (cluster->empty()) return 0.0;

    double sum = 0.0;
    for (const auto& pt : cluster->points)
    {
        if (std::isfinite(pt.intensity)) {
            sum += pt.intensity;  // intensity is already float
        }
    }

    double mean_score = sum / static_cast<double>(cluster->size());
    return mean_score;
}


void CloudCluster::publishBoundingBoxes(const std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto& indices : clusters)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
        for (int idx : indices.indices)
        {
            const auto& pt = cloud->points[idx];
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            cluster->points.push_back(pt);
        }

        // Skip empty clusters
        if (cluster->points.empty()) continue;

        // Bounding Box berechnen auf cluster
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();

        for (const auto& pt : cluster->points)
        {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

            min_x = std::min(min_x, pt.x);
            min_y = std::min(min_y, pt.y);
            min_z = std::min(min_z, pt.z);
            max_x = std::max(max_x, pt.x);
            max_y = std::max(max_y, pt.y);
            max_z = std::max(max_z, pt.z);
        }

        float center_x = (min_x + max_x) / 2.0;
        float center_y = (min_y + max_y) / 2.0;
        float center_z = (min_z + max_z) / 2.0;

        float size_x = std::max(0.01f, max_x - min_x);
        float size_y = std::max(0.01f, max_y - min_y);
        float size_z = std::max(0.01f, max_z - min_z);

        visualization_msgs::msg::Marker box;
        box.header.frame_id = graphNodes->getFrameId();
        box.header.stamp = this->get_clock()->now();
        box.ns = "cloud_cluster/bbox";
        box.id = id++;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;

        box.pose.position.x = center_x;
        box.pose.position.y = center_y;
        box.pose.position.z = center_z;
        box.pose.orientation.w = 1.0;

        box.scale.x = size_x;
        box.scale.y = size_y;
        box.scale.z = size_z;

        box.color.r = 0.0f;
        box.color.g = 1.0f;
        box.color.b = 0.0f;
        box.color.a = 0.3f;

        // box.lifetime = rclcpp::Duration::from_seconds(0.5);

        marker_array.markers.push_back(box);
    }

    markerPub->publish(marker_array);
}

void CloudCluster::timerCallback()
{
    auto t0 = std::chrono::steady_clock::now();

    graphNodes->clear();
    visualization_msgs::msg::MarkerArray all_boxes;
    int globalBoxId = 0;
    int nodeId = 0;

    if (publishBoundingBoxesEnabled) {
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        clear.header.frame_id = graphNodes->getFrameId();
        clear.header.stamp = this->get_clock()->now();
        all_boxes.markers.push_back(clear);
    }

    for (auto& kv : cloudsByInstance) {
        uint16_t inst = kv.first;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in = kv.second;
        if (!cloud_in || cloud_in->empty()) continue;

        // optional voxel filter
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud_in);
        voxel.setLeafSize(static_cast<float>(voxelLeafSize),
                          static_cast<float>(voxelLeafSize),
                          static_cast<float>(voxelLeafSize));
        voxel.filter(*cloud);
        if (cloud->empty()) continue;

        // compute bbox
        float min_x=FLT_MAX,min_y=FLT_MAX,min_z=FLT_MAX;
        float max_x=-FLT_MAX,max_y=-FLT_MAX,max_z=-FLT_MAX;
        for (const auto& pt : cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            min_x = std::min(min_x, pt.x); min_y = std::min(min_y, pt.y); min_z = std::min(min_z, pt.z);
            max_x = std::max(max_x, pt.x); max_y = std::max(max_y, pt.y); max_z = std::max(max_z, pt.z);
        }
        if (!(min_x<max_x && min_y<max_y && min_z<max_z)) continue;

        if (publishBoundingBoxesEnabled) {
            visualization_msgs::msg::Marker box;
            box.header.frame_id = graphNodes->getFrameId();
            box.header.stamp = this->get_clock()->now();
            box.ns = "bbox/inst_" + std::to_string(inst);
            box.id = globalBoxId++;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = 0.5f*(min_x+max_x);
            box.pose.position.y = 0.5f*(min_y+max_y);
            box.pose.position.z = 0.5f*(min_z+max_z);
            box.pose.orientation.w = 1.0;
            box.scale.x = std::max(0.01f, max_x-min_x);
            box.scale.y = std::max(0.01f, max_y-min_y);
            box.scale.z = std::max(0.01f, max_z-min_z);
            box.color.r = (inst * 37 % 255) / 255.0f;
            box.color.g = 1.0f;
            box.color.b = (inst * 73 % 255) / 255.0f;
            box.color.a = 0.3f;
            all_boxes.markers.push_back(box);
        }

        // graph node
        GraphNode node;
        int id = nodeId++;
        node.setId(id);
        auto c = getCentroid(cloud);
        node.setPosition(c);
        double score = getScoreCluster(cloud);
        node.setScore(score);
        graphNodes->addNode(node);
    }

    // publish
    if (publishBoundingBoxesEnabled) 
    {
        markerPub->publish(all_boxes);
    }
    
    graphNodes->publishPosMarkers();
    graphNodes->publishGraphNodeArray();

    auto t1 = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(this->get_logger(), "Per-instance (no clustering) took %.3f s",
                 std::chrono::duration<double>(t1 - t0).count());
}
