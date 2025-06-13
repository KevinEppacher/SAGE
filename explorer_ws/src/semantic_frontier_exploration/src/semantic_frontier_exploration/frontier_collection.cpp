#include "frontier_collection.hpp"


FrontierCollection::FrontierCollection(rclcpp::Node* node) : node(node)
{
    frontier = std::make_shared<Frontier>(node);

    getParameters();

    frontiersPub = node->create_publisher<visualization_msgs::msg::Marker>("frontiers", 1);

    markerPub = node->create_publisher<visualization_msgs::msg::MarkerArray>("cloud_cluster/centroids", 1);

    paramCallbackHandle = node->add_on_set_parameters_callback(
        std::bind(&FrontierCollection::onParameterChange, this, std::placeholders::_1)
    );

    frontierGridPub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("frontier_grid", 10);

    graphNodePub = node->create_publisher<graph_node_msgs::msg::GraphNodeArray>("graph_nodes", 10);

    clock = node->get_clock();

}

FrontierCollection::~FrontierCollection() 
{

}

void FrontierCollection::getParameters()
{
    node->declare_parameter("min_cluster_size", 5);
    node->declare_parameter("max_cluster_size", 150);

    node->get_parameter("min_cluster_size", minClusterSize);
    node->get_parameter("max_cluster_size", maxClusterSize);

    RCLCPP_INFO(node->get_logger(), "Loaded parameters:");
    for (const auto &name : node->list_parameters({}, 10).names)
    {
        auto param = node->get_parameter(name);
        switch (param.get_type())
        {
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                RCLCPP_INFO(node->get_logger(), " - %s: %f", name.c_str(), param.as_double());
                break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                RCLCPP_INFO(node->get_logger(), " - %s: %ld", name.c_str(), param.as_int());
                break;
            case rclcpp::ParameterType::PARAMETER_STRING:
                RCLCPP_INFO(node->get_logger(), " - %s: %s", name.c_str(), param.as_string().c_str());
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
                RCLCPP_INFO(node->get_logger(), " - %s: %s", name.c_str(), param.as_bool() ? "true" : "false");
                break;
            default:
                RCLCPP_INFO(node->get_logger(), " - %s: [unsupported type]", name.c_str());
                break;
        }
    }
}

rcl_interfaces::msg::SetParametersResult FrontierCollection::onParameterChange(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated";

    for (const auto &param : params)
    {
        if (param.get_name() == "min_cluster_size") {
            minClusterSize = param.as_int();
            RCLCPP_INFO(node->get_logger(), "Updated min_cluster_size to %d", minClusterSize);
        } else if (param.get_name() == "max_cluster_size") {
            maxClusterSize = param.as_int();
            RCLCPP_INFO(node->get_logger(), "Updated max_cluster_size to %d", maxClusterSize);
        }
    }

    return result;
}

nav_msgs::msg::OccupancyGrid FrontierCollection::detectFrontierGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid)
{
    nav_msgs::msg::OccupancyGrid frontierGrid;

    frontierGrid.header = grid->header;
    frontierGrid.info = grid->info;

    int width = grid->info.width;
    int height = grid->info.height;
    frontierGrid.data.resize(width * height, -1);

    const auto &data = grid->data;

    auto index = [width](int x, int y) { return y * width + x; };

    for (int y = 1; y < height - 1; ++y)
    {
        for (int x = 1; x < width - 1; ++x)
        {
            int idx = index(x, y);
            if (data[idx] != 0) continue;

            bool is_frontier = false;
            for (int dx : {-1, 1})
            {
                if (data[index(x + dx, y)] == -1) { is_frontier = true; break; }
            }
            for (int dy : {-1, 1})
            {
                if (data[index(x, y + dy)] == -1) { is_frontier = true; break; }
            }

            if (is_frontier)
            {
                frontierGrid.data[idx] = 100;
            }
        }
    }

    frontierGridPub->publish(frontierGrid);

    return frontierGrid;
}

std::vector<Frontier> FrontierCollection::clusterFrontierGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& frontierGrid)
{
    // Output vector containing all clustered frontier groups
    std::vector<Frontier> clusteredFrontiers;

    // Early exit if the grid is invalid or empty
    if (!frontierGrid || frontierGrid->data.empty())
    {
        RCLCPP_WARN(node->get_logger(), "Frontier grid is empty or null.");
        return clusteredFrontiers;
    }

    // Grid metadata
    int width = frontierGrid->info.width;
    int height = frontierGrid->info.height;
    float resolution = frontierGrid->info.resolution;
    float origin_x = frontierGrid->info.origin.position.x;
    float origin_y = frontierGrid->info.origin.position.y;

    const auto &data = frontierGrid->data;

    // Track which cells have already been visited
    std::vector<bool> visited(width * height, false);

    // Helper lambda: converts 2D coordinates to linear array index
    auto index = [width](int x, int y) { return y * width + x; };

    // Helper lambda: checks whether coordinates are inside grid bounds
    auto inBounds = [width, height](int x, int y) 
    {
        return x >= 0 && x < static_cast<int>(width) && y >= 0 && y < static_cast<int>(height);
    };

    // Iterate through each grid cell
    for (int y = 0; y < static_cast<int>(height); ++y)
    {
        for (int x = 0; x < static_cast<int>(width); ++x)
        {
            int idx = index(x, y);

            // Skip if cell is not marked as frontier (value 100) or already visited
            if (data[idx] != 100 || visited[idx]) continue;

            // Start a new cluster using BFS
            std::queue<std::pair<int, int>> queue;
            std::vector<geometry_msgs::msg::Point> cluster;
            queue.push({x, y});
            visited[idx] = true;

            // Perform BFS from the current frontier cell
            while (!queue.empty())
            {
                auto [cx, cy] = queue.front(); queue.pop();

                // Convert grid cell to real-world coordinates
                geometry_msgs::msg::Point pt;
                pt.x = origin_x + cx * resolution;
                pt.y = origin_y + cy * resolution;
                pt.z = 0.0;
                cluster.push_back(pt);

                // Check 8-connected neighbors
                for (int dx = -1; dx <= 1; ++dx)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        if (dx == 0 && dy == 0) continue;
                        int nx = cx + dx;
                        int ny = cy + dy;
                        if (!inBounds(nx, ny)) continue;

                        int nidx = index(nx, ny);
                        if (data[nidx] == 100 && !visited[nidx])
                        {
                            visited[nidx] = true;
                            queue.push({nx, ny});
                        }
                    }
                }
            }

            // If the cluster is big enough, store it as a Frontier
            if (static_cast<int>(cluster.size()) >= minClusterSize && 
                static_cast<int>(cluster.size()) <= maxClusterSize)
            {
                // Create a new Frontier object and set its properties
                Frontier frontier(node);
                geometry_msgs::msg::Point centroid;
                for (const auto& point : cluster)
                {
                    frontier.addPoint(point);
                    centroid.x += point.x;
                    centroid.y += point.y;
                }
                centroid.x /= cluster.size();
                centroid.y /= cluster.size();
                frontier.setCentroid(centroid);
                clusteredFrontiers.push_back(frontier);
                // std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
    }

    return clusteredFrontiers;
}

void FrontierCollection::publishFrontiers(const std::vector<Frontier>& frontiers)
{
    visualization_msgs::msg::Marker point_marker;
    point_marker.header.frame_id = "map";
    point_marker.header.stamp = node->get_clock()->now();
    point_marker.ns = "frontier";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    point_marker.action = visualization_msgs::msg::Marker::ADD;
    point_marker.scale.x = 0.1;
    point_marker.scale.y = 0.1;
    point_marker.scale.z = 0.1;
    point_marker.color.r = 1.0;
    point_marker.color.g = 0.0;
    point_marker.color.b = 0.0;
    point_marker.color.a = 0.8;

    visualization_msgs::msg::Marker centroid_marker;
    centroid_marker.header.frame_id = "map";
    centroid_marker.header.stamp = node->get_clock()->now();
    centroid_marker.ns = "frontier_centroids";
    centroid_marker.id = 1;
    centroid_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    centroid_marker.action = visualization_msgs::msg::Marker::ADD;
    centroid_marker.scale.x = 0.2;
    centroid_marker.scale.y = 0.2;
    centroid_marker.scale.z = 0.2;
    centroid_marker.color.r = 0.0;
    centroid_marker.color.g = 1.0;
    centroid_marker.color.b = 0.0;
    centroid_marker.color.a = 1.0;

    int text_id = 100;  // unique ID for each text marker

    for (const auto& f : frontiers)
    {
        for (const auto& p : f.getFrontier().points)
        {
            point_marker.points.push_back(p);
        }

        geometry_msgs::msg::Point centroid = f.getCentroid();
        centroid_marker.points.push_back(centroid);

        // Add text marker for this frontier
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = node->get_clock()->now();
        text_marker.ns = "frontier_scores";
        text_marker.id = text_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position = centroid;
        text_marker.pose.position.z += 0.3;  // lift text above point
        text_marker.scale.z = 0.2;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "Score: " + std::to_string(f.getScore());

        frontiersPub->publish(text_marker);
    }

    // Publish both markers
    frontiersPub->publish(point_marker);
    frontiersPub->publish(centroid_marker);
}

void FrontierCollection::publishGraphNodes(const std::vector<Frontier>& frontiers)
{
    graph_node_msgs::msg::GraphNodeArray msg;

    for(const auto& frontier : frontiers)
    {
        graph_node_msgs::msg::GraphNode graphNode;
        graphNode.header.frame_id = "map";
        graphNode.header.stamp = node->get_clock()->now();
        graphNode.position.x = frontier.getCentroid().x;
        graphNode.position.y = frontier.getCentroid().y;
        graphNode.position.z = frontier.getCentroid().z;
        graphNode.score = frontier.getScore();
        msg.nodes.push_back(graphNode);
    }

    // publishGraphNodesMarker(msg);

    graphNodePub->publish(msg);
}

// void FrontierCollection::publishGraphNodesMarker(const graph_node_msgs::msg::GraphNodeArray& msg)
// {
//     visualization_msgs::msg::MarkerArray markers;
//     int id = 0;

//     // 1. Min/Max Score berechnen
//     double min_score = std::numeric_limits<double>::max();
//     double max_score = std::numeric_limits<double>::lowest();
//     int max_index = -1;

//     for (size_t i = 0; i < msg.nodes.size(); ++i)
//     {
//         double score = msg.nodes[i].getScore();
//         if (score < min_score) min_score = score;
//         if (score > max_score) {
//             max_score = score;
//             max_index = static_cast<int>(i);
//         }
//     }

//     for (const auto& node : msg.nodes)
//     {
//         auto pos = node.getPosition();
//         auto score = node.getScore();

//         double normalized = (max_score > min_score) ?
//             (score - min_score) / (max_score - min_score) : 0.0;

//         // SPHERE Marker
//         visualization_msgs::msg::Marker sphere;
//         sphere.header.frame_id = "map";
//         sphere.header.stamp = clock->now();
//         sphere.ns = "graph_nodes";
//         sphere.id = id++;
//         sphere.type = visualization_msgs::msg::Marker::SPHERE;
//         sphere.action = visualization_msgs::msg::Marker::ADD;
//         sphere.pose.position = pos;
//         sphere.pose.orientation.w = 1.0;
//         sphere.scale.x = 0.1;
//         sphere.scale.y = 0.1;
//         sphere.scale.z = 0.1;
//         sphere.color.r = 1.0 - normalized;
//         sphere.color.g = normalized;
//         sphere.color.b = 0.0;
//         sphere.color.a = 0.8;
//         markers.markers.push_back(sphere);

//         // TEXT Marker
//         visualization_msgs::msg::Marker text;
//         text.header.frame_id = frameId;
//         text.header.stamp = clock->now();
//         text.ns = "graph_nodes_text";
//         text.id = id++;
//         text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         text.action = visualization_msgs::msg::Marker::ADD;
//         text.pose.position = pos;
//         text.pose.position.z += 0.15;
//         text.pose.orientation.w = 1.0;
//         text.scale.z = 0.1;
//         text.color.r = 1.0;
//         text.color.g = 1.0;
//         text.color.b = 1.0;
//         text.color.a = 1.0;

//         std::ostringstream stream;
//         stream << std::fixed << std::setprecision(2) << score;
//         text.text = stream.str();
//         markers.markers.push_back(text);
//     }

//     // Highlight Marker für Node mit höchstem Score
//     if (max_index >= 0 && max_index < static_cast<int>(nodes.size()))
//     {
//         auto pos = nodes[max_index].getPosition();
//         visualization_msgs::msg::Marker star;
//         star.header.frame_id = "map";
//         star.header.stamp = clock->now();
//         star.ns = "graph_nodes_best";
//         star.id = id++;
//         star.type = visualization_msgs::msg::Marker::SPHERE;
//         star.action = visualization_msgs::msg::Marker::ADD;
//         star.pose.position = pos;
//         star.pose.orientation.w = 1.0;
//         star.scale.x = 0.3;
//         star.scale.y = 0.3;
//         star.scale.z = 0.3;
//         star.color.r = 1.0;
//         star.color.g = 1.0;
//         star.color.b = 0.0;
//         star.color.a = 1.0;
//         markers.markers.push_back(star);
//     }

//     markerPub->publish(markers);
// }

void FrontierCollection::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    latestClock = msg->clock;
}