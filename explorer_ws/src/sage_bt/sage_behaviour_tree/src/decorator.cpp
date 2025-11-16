#include "sage_behaviour_tree/decorator.hpp"
#include "sage_behaviour_tree/colors.hpp"

// ============================ KeepRunningUntilObjectFound ============================ //

KeepRunningUntilObjectFound::KeepRunningUntilObjectFound(
    const std::string& name,
    const BT::NodeConfiguration& config)
    : BT::DecoratorNode(name, config)
{
}

BT::PortsList KeepRunningUntilObjectFound::providedPorts()
{
    return {
        BT::InputPort<bool>("object_found", false, "True if target object was detected"),
        BT::InputPort<bool>("any_exploration_nodes", true, "True if frontiers or exploration nodes remain")
    };
}

BT::NodeStatus KeepRunningUntilObjectFound::tick()
{
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    if (!initialized)
    {
        if (!getInput("object_found", objectFound))
        {
            objectFound = false;
            setOutput("object_found", objectFound);
        }

        if (!getInput("any_exploration_nodes", anyExplorationNodes))
        {
            anyExplorationNodes = true;
            setOutput("any_exploration_nodes", anyExplorationNodes);
        }

        initialized = true;
    }

    getInput("object_found", objectFound);
    getInput("any_exploration_nodes", anyExplorationNodes);

    if (objectFound)
    {
        RCLCPP_INFO(rclcpp::get_logger("KeepRunningUntilObjectFound"),
                    GREEN "[%s] Object found → SUCCESS" RESET, name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    if (!anyExplorationNodes)
    {
        RCLCPP_WARN(rclcpp::get_logger("KeepRunningUntilObjectFound"),
                    RED "[%s] No exploration nodes remaining → FAILURE" RESET, name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    const BT::NodeStatus childStatus = child_node_->executeTick();

    if (childStatus == BT::NodeStatus::SUCCESS || childStatus == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("KeepRunningUntilObjectFound"),
                             steady_clock, 2000,
                             ORANGE "[%s] Child finished, continuing loop (RUNNING)" RESET,
                             name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("KeepRunningUntilObjectFound"),
    //                      steady_clock, 2000,
    //                      ORANGE "[%s] Running child tick..." RESET,
    //                      name().c_str());
    return childStatus;
}

// ============================ ForEachEvaluationPrompt ============================ //

ForEachEvaluationPrompt::ForEachEvaluationPrompt(
    const std::string &name,
    const BT::NodeConfig &config,
    const rclcpp::Node::SharedPtr &nodePtr)
    : BT::DecoratorNode(name, config),
      node(nodePtr)
{
    // Declare parameters if needed
    node->declare_parameter("for_each_evaluation_prompt.evaluation_event_topic",
                            "/evaluator/dashboard/event");
    node->declare_parameter("for_each_evaluation_prompt.iteration_status_topic",
                            "/evaluator/dashboard/iteration_status");
    node->declare_parameter("for_each_evaluation_prompt.debug", false);

    evaluation_event_topic_ =
        node->get_parameter("for_each_evaluation_prompt.evaluation_event_topic").as_string();
    iteration_status_topic_ =
        node->get_parameter("for_each_evaluation_prompt.iteration_status_topic").as_string();
    debug_flag =
        node->get_parameter("for_each_evaluation_prompt.debug").as_bool();
}


BT::PortsList ForEachEvaluationPrompt::providedPorts()
{
    return {
        BT::OutputPort<std::string>("target_object"),
        BT::OutputPort<std::string>("save_image_path")
    };
}

BT::NodeStatus ForEachEvaluationPrompt::tick()
{
    initCommsFromPorts();

    if (!subscribed)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             ORANGE "[%s] Waiting for %s..." RESET,
                             name().c_str(), evaluation_event_topic_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    if (currentIndex >= promptTexts.size())
    {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[%s] All prompts processed → SUCCESS" RESET,
                    name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    std::string currentPrompt = promptTexts[currentIndex];

    // normalize class folder name (no spaces or commas)
    std::string classFolder = currentPrompt;
    std::replace(classFolder.begin(), classFolder.end(), ' ', '_');
    std::replace(classFolder.begin(), classFolder.end(), ',', '_');

    // build canonical detection path
    std::string detectionDir = baseSavePath + "detections/" + classFolder + "/";
    if (!std::filesystem::exists(detectionDir))
        std::filesystem::create_directories(detectionDir);

    // image name pattern
    char filename[64];
    snprintf(filename, sizeof(filename), "detection_%04zu.png", currentIndex + 1);
    std::string saveFile = detectionDir + filename;

    // export BT ports
    setOutput("target_object", currentPrompt);
    setOutput("save_image_path", saveFile);

    BT::NodeStatus childStatus = child_node_->executeTick();

    if (childStatus == BT::NodeStatus::RUNNING)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
                            ORANGE "[%s] Processing prompt '%s'..." RESET,
                            name().c_str(), currentPrompt.c_str());
        return BT::NodeStatus::RUNNING;
    }

    // report back to dashboard
    std_msgs::msg::String msg;
    msg.data = "[ForEachEvaluationPrompt] Object '" + currentPrompt +
            "' finished with status: " +
            (childStatus == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    statusPublisher->publish(msg);

    // optional: write quick metadata JSON stub
    std::ofstream metaFile(detectionDir + "detection_meta.json", std::ios::app);
    if (metaFile)
    {
        metaFile << "{ \"object\": \"" << currentPrompt
                << "\", \"status\": \"" << (childStatus == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE")
                << "\", \"image\": \"" << filename << "\" }\n";
    }

    RCLCPP_INFO(node->get_logger(),
                (childStatus == BT::NodeStatus::SUCCESS
                    ? GREEN "[%s] Prompt '%s' → SUCCESS" RESET
                    : RED "[%s] Prompt '%s' → FAILURE" RESET),
                name().c_str(), currentPrompt.c_str());

    currentIndex++;
    return BT::NodeStatus::RUNNING;
}

void ForEachEvaluationPrompt::callbackEvent(const EvaluationEvent::SharedPtr msg)
{
    if (subscribed)
        return;
    subscribed = true;

    baseSavePath = msg->save_path;  // e.g. /app/src/sage_evaluator/sage_evaluator/data/scene/EXP_001/E01/
    if (!baseSavePath.empty() && baseSavePath.back() != '/')
        baseSavePath += '/';

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] === Evaluation Configuration Received ===" RESET,
                name().c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Experiment: %s" RESET,
                name().c_str(), msg->experiment_id.c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Scene: %s" RESET,
                name().c_str(), msg->scene.c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Save Path: %s" RESET,
                name().c_str(), msg->save_path.c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Episode: %s" RESET,
                name().c_str(), msg->episode_id.c_str());
    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Prompts in this run:" RESET,
                name().c_str());

    for (const auto &prompt : msg->prompt_list.prompt_list)
    {
        RCLCPP_INFO(node->get_logger(),
                    ORANGE "[%s]   - %s" RESET,
                    name().c_str(), prompt.text_query.c_str());
        promptTexts.push_back(prompt.text_query);
    }

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] ==========================================" RESET,
                name().c_str());
}

void ForEachEvaluationPrompt::initCommsFromPorts()
{
    if (commReady) return;

    // Subscriber must match dashboard publisher QoS
    rclcpp::QoS event_qos(1);
    event_qos.transient_local();
    event_qos.reliable();

    subscriber = node->create_subscription<EvaluationEvent>(
        evaluation_event_topic_,
        event_qos,
        std::bind(&ForEachEvaluationPrompt::callbackEvent, this, std::placeholders::_1)
    );

    // Publisher: normal volatile reliable
    rclcpp::QoS status_qos(10);
    status_qos.reliable();

    statusPublisher = node->create_publisher<std_msgs::msg::String>(
        iteration_status_topic_,
        status_qos
    );

    if (debug_flag)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "[%s] Using topics: event='%s', status='%s'",
            name().c_str(), evaluation_event_topic_.c_str(), iteration_status_topic_.c_str());
    }

    commReady = true;
}


// ============================ ApproachPoseAdjustor ============================ //

ApproachPoseAdjustor::ApproachPoseAdjustor(const std::string &name,
                                           const BT::NodeConfiguration &config,
                                           rclcpp::Node::SharedPtr nodePtr)
    : BT::DecoratorNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node))
{
    std::string markerTopic = "approach_pose_adjustor/markers";
    // Optional parameters (declare/get if you like)
    if (!node->has_parameter("approach_pose_adjustor.robot_radius"))
        node->declare_parameter<double>("approach_pose_adjustor.robot_radius", robotRadius);
    if (!node->has_parameter("approach_pose_adjustor.cost_threshold"))
        node->declare_parameter<int>("approach_pose_adjustor.cost_threshold", costThreshold);
    if (!node->has_parameter("approach_pose_adjustor.allow_unknown"))
        node->declare_parameter<bool>("approach_pose_adjustor.allow_unknown", allowUnknown);
    if (!node->has_parameter("approach_pose_adjustor.angular_step_deg"))
        node->declare_parameter<double>("approach_pose_adjustor.angular_step_deg", angularStepDeg);
    if (!node->has_parameter("approach_pose_adjustor.radial_samples"))
        node->declare_parameter<int>("approach_pose_adjustor.radial_samples", radialSamples);
    if (!node->has_parameter("approach_pose_adjustor.line_step"))
        node->declare_parameter<double>("approach_pose_adjustor.line_step", lineStep);
    if (!node->has_parameter("approach_pose_adjustor.marker_frame"))
        node->declare_parameter<std::string>("approach_pose_adjustor.marker_frame", markerFrame);
    if (!node->has_parameter("approach_pose_adjustor.debug_markers"))
        node->declare_parameter<bool>("approach_pose_adjustor.debug_markers", debugMarkers);
    if (!node->has_parameter("approach_pose_adjustor.marker_topic"))
        node->declare_parameter<std::string>("approach_pose_adjustor.marker_topic", markerFrame);

    robotRadius   = node->get_parameter("approach_pose_adjustor.robot_radius").as_double();
    costThreshold = node->get_parameter("approach_pose_adjustor.cost_threshold").as_int();
    allowUnknown  = node->get_parameter("approach_pose_adjustor.allow_unknown").as_bool();
    angularStepDeg= node->get_parameter("approach_pose_adjustor.angular_step_deg").as_double();
    radialSamples = node->get_parameter("approach_pose_adjustor.radial_samples").as_int();
    lineStep      = node->get_parameter("approach_pose_adjustor.line_step").as_double();
    markerFrame   = node->get_parameter("approach_pose_adjustor.marker_frame").as_string();
    debugMarkers  = node->get_parameter("approach_pose_adjustor.debug_markers").as_bool();

    markerPub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        markerTopic, 10);
}

BT::PortsList ApproachPoseAdjustor::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node", "Input target node"),
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("reachable_graph_node", "Adjusted reachable node"),
        BT::InputPort<double>("search_radius", 2.5, "Maximum approach distance (m)")
    };
}

BT::NodeStatus ApproachPoseAdjustor::tick()
{
    getInput("search_radius", searchRadius);

    auto inputNodeRes = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node");
    if (!inputNodeRes || !inputNodeRes.value())
    {
        RCLCPP_WARN(node->get_logger(),
                    RED "[%s] No input graph_node provided." RESET,
                    name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    const auto target = *inputNodeRes.value();
    geometry_msgs::msg::Pose robotPose{};
    if (!robot->getPose(robotPose))
    {
        RCLCPP_WARN(node->get_logger(),
                    RED "[%s] No robot pose." RESET,
                    name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Compute only once per activation
    if (!haveCachedReachable)
    {
        graph_node_msgs::msg::GraphNode reachable = target;
        const bool found = findReachablePoint(target, reachable);

        if (!found)
        {
            RCLCPP_WARN(node->get_logger(),
                        YELLOW "[%s] No reachable approach pose found inside radius %.2f m → FAILURE" RESET,
                        name().c_str(), searchRadius);
            if (debugMarkers)
                publishMarkers(robotPose, target, nullptr, "ApproachPoseAdjustor");
            setOutput("reachable_graph_node", inputNodeRes.value());
        }

        cachedReachable = reachable;
        haveCachedReachable = true;

        RCLCPP_INFO(node->get_logger(),
                    CYAN "[%s] Cached reachable node at (%.2f, %.2f)" RESET,
                    name().c_str(),
                    cachedReachable.position.x, cachedReachable.position.y);
    }

    // Always publish markers so RViz stays updated
    if (debugMarkers)
        publishMarkers(robotPose, target,
                       haveCachedReachable ? &cachedReachable : nullptr,
                       "ApproachPoseAdjustor");

    // Always keep output port updated while running
    if (haveCachedReachable)
        setOutput("reachable_graph_node",
                  std::make_shared<graph_node_msgs::msg::GraphNode>(cachedReachable));

    // Tick child node
    lastChildStatus = child_node_->executeTick();

    // When done, reset cache
    if (lastChildStatus != BT::NodeStatus::RUNNING)
        haveCachedReachable = false;

    return lastChildStatus;
}

bool ApproachPoseAdjustor::findReachablePoint(const graph_node_msgs::msg::GraphNode &target,
                                              graph_node_msgs::msg::GraphNode &reachable)
{
    lastSamples.clear();
    lastTarget = target;
    lastReachableValid = false;

    auto gridPtr = robot->getGlobalCostmap();
    if (!gridPtr)
    {
        reachable = target;
        lastReachable = target;
        lastReachableValid = true;
        return true;
    }
    const auto &grid = *gridPtr;

    geometry_msgs::msg::Pose robotPose{};
    if (!robot->getPose(robotPose))
        return false;

    const double tx = target.position.x;
    const double ty = target.position.y;
    double baseAngle = std::atan2(robotPose.position.y - ty, robotPose.position.x - tx);

    std::vector<double> radii;
    radii.push_back(0.0);
    if (radialSamples < 1) radialSamples = 1;
    for (int i = 0; i < radialSamples; ++i)
        radii.push_back(searchRadius * (i + 1) / static_cast<double>(radialSamples));

    const double stepRad = angularStepDeg * M_PI / 180.0;
    auto angles = [&]() {
        std::vector<double> seq{baseAngle};
        for (int k = 1; k <= static_cast<int>(M_PI / stepRad) + 1; ++k)
        {
            seq.push_back(baseAngle + k * stepRad);
            seq.push_back(baseAngle - k * stepRad);
        }
        return seq;
    }();

    // Try candidates
    for (double r : radii)
    {
        for (double ang : angles)
        {
            double cx = tx + r * std::cos(ang);
            double cy = ty + r * std::sin(ang);

            geometry_msgs::msg::Point p;
            p.x = cx; p.y = cy; p.z = 0.05;
            lastSamples.push_back(p);

            if (!isFootprintFree(grid, cx, cy, robotRadius))
                continue;
            if (!rayPathAcceptable(grid, robotPose.position.x, robotPose.position.y, cx, cy))
                continue;

            reachable = target;
            reachable.position.x = cx;
            reachable.position.y = cy;
            reachable.position.z = 0.0;
            lastReachable = reachable;
            lastReachableValid = true;
            return true;
        }
    }
    return false;
}

bool ApproachPoseAdjustor::isFootprintFree(const nav_msgs::msg::OccupancyGrid &grid,
                                           double x, double y,
                                           double radius) const
{
    const auto &info = grid.info;
    const double res = info.resolution;

    // Compute a square in map coords that bounds the circle, then test cells inside circle.
    int min_mx, min_my, max_mx, max_my;
    {
        int mx0, my0, mx1, my1, mx2, my2, mx3, my3;
        worldToMap(grid, x - radius, y - radius, mx0, my0);
        worldToMap(grid, x + radius, y - radius, mx1, my1);
        worldToMap(grid, x - radius, y + radius, mx2, my2);
        worldToMap(grid, x + radius, y + radius, mx3, my3);
        min_mx = std::min(std::min(mx0, mx1), std::min(mx2, mx3));
        min_my = std::min(std::min(my0, my1), std::min(my2, my3));
        max_mx = std::max(std::max(mx0, mx1), std::max(mx2, mx3));
        max_my = std::max(std::max(my0, my1), std::max(my2, my3));
    }

    // Clamp to bounds
    min_mx = std::max(min_mx, 0);
    min_my = std::max(min_my, 0);
    max_mx = std::min<int>(max_mx, info.width - 1);
    max_my = std::min<int>(max_my, info.height - 1);

    const double r2 = radius * radius;

    for (int my = min_my; my <= max_my; ++my)
    {
        for (int mx = min_mx; mx <= max_mx; ++mx)
        {
            const double wx = info.origin.position.x + (mx + 0.5) * res;
            const double wy = info.origin.position.y + (my + 0.5) * res;

            const double dx = wx - x;
            const double dy = wy - y;
            if (dx*dx + dy*dy > r2)
                continue; // outside circle

            const int idx = my * info.width + mx;
            if (!gridValueAcceptable(grid.data[idx]))
                return false; // cell is too costly/unknown
        }
    }
    return true;
}

bool ApproachPoseAdjustor::rayPathAcceptable(const nav_msgs::msg::OccupancyGrid &grid,
                                             double x0, double y0,
                                             double x1, double y1) const
{
    const auto &info = grid.info;
    const double res = info.resolution;

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6)
        return true;

    const int steps = std::max(1, static_cast<int>(dist / std::max(lineStep, res * 0.5)));
    const double ux = dx / steps;
    const double uy = dy / steps;

    for (int i = 0; i <= steps; ++i)
    {
        const double wx = x0 + i * ux;
        const double wy = y0 + i * uy;

        int mx, my;
        if (!worldToMap(grid, wx, wy, mx, my))
            return false; // outside map → treat as blocked

        const int idx = my * info.width + mx;
        if (!gridValueAcceptable(grid.data[idx]))
            return false; // hit obstacle/high cost/unknown
    }
    return true;
}

void ApproachPoseAdjustor::publishMarkers(const geometry_msgs::msg::Pose &robotPose,
                                          const graph_node_msgs::msg::GraphNode &target,
                                          const graph_node_msgs::msg::GraphNode *reachable,
                                          const std::string &ns)
{
    visualization_msgs::msg::MarkerArray arr;
    const rclcpp::Time now = node->now();

    auto make_marker = [&](int id, int type, const std::string &nspace) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = markerFrame;
        m.header.stamp = now;
        m.ns = nspace;
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.lifetime = rclcpp::Duration::from_seconds(2.0);
        return m;
    };

    // Original target
    {
        auto m = make_marker(1, visualization_msgs::msg::Marker::SPHERE, ns);
        m.scale.x = m.scale.y = m.scale.z = 0.25;
        m.color.r = 1.0; m.color.a = 0.9;
        m.pose.position.x = target.position.x;
        m.pose.position.y = target.position.y;
        arr.markers.push_back(m);
    }

    // Sample points (gray)
    int id = 10;
    for (const auto &p : lastSamples)
    {
        auto s = make_marker(id++, visualization_msgs::msg::Marker::SPHERE, ns + "_samples");
        s.scale.x = s.scale.y = s.scale.z = 0.05;
        s.color.r = 0.6; s.color.g = 0.6; s.color.b = 0.6; s.color.a = 0.5;
        s.pose.position = p;
        arr.markers.push_back(s);
    }

    // Reachable candidate
    if (reachable)
    {
        auto m = make_marker(2, visualization_msgs::msg::Marker::SPHERE, ns);
        m.scale.x = m.scale.y = m.scale.z = 0.25;
        m.color.g = 1.0; m.color.a = 0.9;
        m.pose.position.x = reachable->position.x;
        m.pose.position.y = reachable->position.y;
        arr.markers.push_back(m);

        // Line robot → reachable
        auto l = make_marker(3, visualization_msgs::msg::Marker::LINE_STRIP, ns);
        l.scale.x = 0.03;
        l.color.b = 1.0; l.color.a = 0.8;
        geometry_msgs::msg::Point p0, p1;
        p0.x = robotPose.position.x; p0.y = robotPose.position.y; p0.z = 0.05;
        p1.x = reachable->position.x; p1.y = reachable->position.y; p1.z = 0.05;
        l.points.push_back(p0);
        l.points.push_back(p1);
        arr.markers.push_back(l);

        // Virtual footprint at reachable pose
        auto f = make_marker(4, visualization_msgs::msg::Marker::CYLINDER, ns);
        f.scale.x = f.scale.y = robotRadius * 2.0;
        f.scale.z = 0.05;
        f.color.r = 1.0; f.color.g = 1.0; f.color.b = 0.0; f.color.a = 0.5;
        f.pose.position.x = reachable->position.x;
        f.pose.position.y = reachable->position.y;
        f.pose.position.z = 0.01;
        arr.markers.push_back(f);
    }

    markerPub->publish(arr);
}

// ============================ SageBtOrchestrator ============================ //

SageBtOrchestrator::SageBtOrchestrator(
    const std::string &name,
    const BT::NodeConfig &config,
    const rclcpp::Node::SharedPtr &nodePtr)
    : BT::DecoratorNode(name, config),
      node(nodePtr)
{
    declareIfNotDeclared("sage_bt.startup.required_topics", rclcpp::ParameterValue(std::vector<std::string>{}));
    declareIfNotDeclared("sage_bt.startup.required_services", rclcpp::ParameterValue(std::vector<std::string>{}));
    loadParameters();

    startupService = node->create_service<sage_bt_msgs::srv::StartupCheck>(
        "startup_check",
        std::bind(&SageBtOrchestrator::onStartupRequest, this, std::placeholders::_1, std::placeholders::_2));

    // Create action server for ExecutePrompt
    actionServer = rclcpp_action::create_server<ExecutePrompt>(
        node,
        "execute_prompt",
        std::bind(&SageBtOrchestrator::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SageBtOrchestrator::handleCancel, this, std::placeholders::_1),
        std::bind(&SageBtOrchestrator::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(),
            CYAN "[%s] SageBtOrchestrator READY → Action server '/execute_prompt' active." RESET,
            this->name().c_str());
}

void SageBtOrchestrator::declareIfNotDeclared(
    const std::string &param_name,
    const rclcpp::ParameterValue &default_value)
{
    if (!node->has_parameter(param_name))
        node->declare_parameter(param_name, default_value);
}

BT::PortsList SageBtOrchestrator::providedPorts()
{
    return {
        BT::OutputPort<std::string>("target_object"),
        BT::OutputPort<std::string>("image_path"),
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "detected_graph_node", "Best detected GraphNode")
    };
}

void SageBtOrchestrator::loadParameters()
{
    required_topics = node->get_parameter("sage_bt.startup.required_topics").as_string_array();
    required_services = node->get_parameter("sage_bt.startup.required_services").as_string_array();
}

// =======================================
// STARTUP CHECK CALLBACK
// =======================================

void SageBtOrchestrator::onStartupRequest(
    const std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Request>,
    std::shared_ptr<sage_bt_msgs::srv::StartupCheck::Response> resp)
{
    std::stringstream report;
    bool all_ok = true;

    for (const auto &t : required_topics)
    {
        if (node->get_topic_names_and_types().count(t) == 0)
        {
            all_ok = false;
            report << "Missing topic: " << t << "\n";
        }
    }

    for (const auto &s : required_services)
    {
        if (node->get_service_names_and_types().count(s) == 0)
        {
            all_ok = false;
            report << "Missing service: " << s << "\n";
        }
    }

    resp->ready = all_ok;
    resp->report = report.str();

    if (all_ok)
    {
        startupDone = true;
        RCLCPP_INFO(node->get_logger(),
            GREEN "[%s] StartupCheck SUCCESS → System ready." RESET,
            name().c_str());
    }
    else
    {
        RCLCPP_WARN(node->get_logger(),
            RED "[%s] StartupCheck FAILED:\n%s" RESET,
            name().c_str(), resp->report.c_str());
    }
}

// =======================================
// ACTION SERVER CALLBACKS
// =======================================

rclcpp_action::GoalResponse SageBtOrchestrator::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecutePrompt::Goal> goal)
{
    if (!startupDone)
    {
        RCLCPP_WARN(node->get_logger(),
            ORANGE "[%s] Received ExecutePrompt goal but startup not ready → rejecting." RESET,
            name().c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (activeGoal)
    {
        RCLCPP_WARN(node->get_logger(),
            ORANGE "[%s] Another goal is active → rejecting new one." RESET,
            name().c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(node->get_logger(),
        CYAN "[%s] New ExecutePrompt goal: '%s'" RESET,
        name().c_str(), goal->prompt.c_str());

    currentPrompt = goal->prompt;
    currentSavePath = goal->save_directory;
    activeGoal = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SageBtOrchestrator::handleCancel(
    const std::shared_ptr<GoalHandle>)
{
    RCLCPP_INFO(node->get_logger(),
        ORANGE "[%s] Goal cancelled." RESET, name().c_str());
    activeGoal = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SageBtOrchestrator::handleAccepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread{std::bind(&SageBtOrchestrator::executeGoal, this, goal_handle)}.detach();
}

void SageBtOrchestrator::executeGoal(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecutePrompt::Result>();

    // Store inputs from goal
    currentPrompt   = goal->prompt;
    currentSavePath = goal->save_directory;
    double timeoutSec = goal->timeout * 60.0;

    // Prepare BT outputs
    setOutput("target_object", currentPrompt);
    setOutput("image_path", currentSavePath);

    RCLCPP_INFO(node->get_logger(),
        ORANGE "[%s] Executing BT subtree for prompt '%s' (timeout %.1f min = %.0fs)..." RESET,
        this->name().c_str(), currentPrompt.c_str(), goal->timeout, timeoutSec);

    // --- Main execution loop ---
    rclcpp::Rate loop_rate(10); // 10 Hz
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Time start_time = node->now();

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        // ---- Handle cancel ----
        if (goal_handle->is_canceling())   // <<=== this is the critical line
        {
            RCLCPP_WARN(node->get_logger(),
                YELLOW "[%s] Goal canceled by client. Halting BT subtree..." RESET,
                this->name().c_str());

            // Halt all running children in BT
            this->haltChild();

            result->result = false;
            result->confidence_score = 0.0f;
            goal_handle->canceled(result);
            activeGoal = false;
            return;
        }

        // ---- Handle timeout ----
        double elapsed = (node->now() - start_time).seconds();
        if (timeoutSec > 0.0 && elapsed > timeoutSec)
        {
            RCLCPP_ERROR(node->get_logger(),
                RED "[%s] TIMEOUT reached after %.2fs (limit %.2fs) → aborting." RESET,
                this->name().c_str(), elapsed, timeoutSec);

            this->haltChild();

            result->result = false;
            result->confidence_score = 0.0f;
            goal_handle->abort(result);
            activeGoal = false;
            return;
        }

        // ---- Tick BT ----
        status = child_node_->executeTick();

        // ---- Feedback ----
        ExecutePrompt::Feedback feedback;
        feedback.active_node = this->name();
        feedback.status = (status == BT::NodeStatus::RUNNING);
        feedback.log = "Ticking subtree...";
        goal_handle->publish_feedback(std::make_shared<ExecutePrompt::Feedback>(feedback));

        loop_rate.sleep();
    }

    // ---- Finalize ----
    bool success = (status == BT::NodeStatus::SUCCESS);
    result->result = success;

    // Try to get detected_graph_node to extract confidence
    std::shared_ptr<graph_node_msgs::msg::GraphNode> detected_node;
    if (getInput("detected_graph_node", detected_node) && detected_node)
    {
        result->confidence_score = static_cast<float>(detected_node->score);
        RCLCPP_INFO(node->get_logger(),
            GREEN "[%s] Retrieved confidence score from detected GraphNode: %.3f" RESET,
            this->name().c_str(), result->confidence_score);
    }
    else
    {
        result->confidence_score = success ? 1.0f : 0.0f;
        RCLCPP_WARN(node->get_logger(),
            YELLOW "[%s] No detected_graph_node found → using default confidence %.2f" RESET,
            this->name().c_str(), result->confidence_score);
    }

    // Send result
    if (success)
    {
        RCLCPP_INFO(node->get_logger(),
            GREEN "[%s] BT finished SUCCESS for '%s'." RESET,
            this->name().c_str(), currentPrompt.c_str());
        goal_handle->succeed(result);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(),
            RED "[%s] BT finished FAILURE for '%s'." RESET,
            this->name().c_str(), currentPrompt.c_str());
        goal_handle->abort(result);
    }

    activeGoal = false;
}

// =======================================
// DECORATOR TICK
// =======================================

BT::NodeStatus SageBtOrchestrator::tick()
{
    static rclcpp::Clock clock(RCL_STEADY_TIME);

    // Wait for system readiness
    if (!startupDone)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 2000,
            ORANGE "[%s] Waiting for StartupCheck..." RESET,
            this->name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    // Only tick subtree if an action goal is active
    if (!activeGoal)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 3000,
            ORANGE "[%s] Idle → waiting for ExecutePrompt goal." RESET,
            this->name().c_str());
        return BT::NodeStatus::RUNNING;  // or RUNNING if your BT framework prefers
    }

    // Execute the child subtree while goal is active
    BT::NodeStatus status = child_node_->executeTick();

    if (status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(),
            GREEN "[%s] Child subtree → SUCCESS." RESET, this->name().c_str());
    }
    else if (status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_ERROR(node->get_logger(),
            RED "[%s] Child subtree → FAILURE." RESET, this->name().c_str());
    }
    else
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 2000,
            ORANGE "[%s] Child subtree RUNNING..." RESET,
            this->name().c_str());
    }

    return status;
}
