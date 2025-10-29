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

ForEachEvaluationPrompt::ForEachEvaluationPrompt(const std::string &name,
                                                 const BT::NodeConfig &config,
                                                 const rclcpp::Node::SharedPtr &nodePtr)
    : BT::DecoratorNode(name, config),
      node(nodePtr)
{
}

BT::PortsList ForEachEvaluationPrompt::providedPorts()
{
    return {
        BT::OutputPort<std::string>("targetObject"),
        BT::InputPort<std::string>("evaluation_event_topic", "/evaluation/event",
                                   "Topic to subscribe for EvaluationEvent"),
        BT::InputPort<std::string>("iteration_status_topic", "/evaluation/iteration_status",
                                   "Topic to publish iteration status")
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
    setOutput("targetObject", currentPrompt);

    BT::NodeStatus childStatus = child_node_->executeTick();

    if (childStatus == BT::NodeStatus::RUNNING)
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
                             ORANGE "[%s] Processing prompt '%s'..." RESET,
                             name().c_str(), currentPrompt.c_str());
        return BT::NodeStatus::RUNNING;
    }

    std_msgs::msg::String msg;
    msg.data = "[ForEachEvaluationPrompt] Object '" + currentPrompt +
               "' finished with status: " +
               (childStatus == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    statusPublisher->publish(msg);

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

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] === Evaluation Configuration Received ===" RESET,
                name().c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Experiment: %s" RESET,
                name().c_str(), msg->experiment_id.c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Episode: %s" RESET,
                name().c_str(), msg->episode_id.c_str());
    RCLCPP_INFO(node->get_logger(), ORANGE "[%s] Phase: %s" RESET,
                name().c_str(), msg->phase.c_str());
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
    if (commReady)
        return;

    (void)getInput<std::string>("evaluation_event_topic", evaluation_event_topic_);
    (void)getInput<std::string>("iteration_status_topic", iteration_status_topic_);

    rclcpp::QoS qos(1);
    qos.transient_local();
    subscriber = node->create_subscription<EvaluationEvent>(
        evaluation_event_topic_, qos,
        std::bind(&ForEachEvaluationPrompt::callbackEvent, this, std::placeholders::_1));

    statusPublisher = node->create_publisher<std_msgs::msg::String>(
        iteration_status_topic_, 10);

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Using topics: event='%s', status='%s'" RESET,
                name().c_str(), evaluation_event_topic_.c_str(), iteration_status_topic_.c_str());

    commReady = true;
}

// ============================ ApproachPoseAdjustor ============================ //

ApproachPoseAdjustor::ApproachPoseAdjustor(const std::string &name,
                                           const BT::NodeConfiguration &config,
                                           rclcpp::Node::SharedPtr nodePtr)
    : BT::DecoratorNode(name, config),
      node(std::move(nodePtr)),
      robot(std::make_shared<Robot>(node))
{}

BT::PortsList ApproachPoseAdjustor::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node", "Input target node"),
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("reachable_graph_node", "Adjusted reachable node"),
        BT::InputPort<double>("approach_radius", 1.0, "Maximum projection distance (m)")
    };
}

BT::NodeStatus ApproachPoseAdjustor::tick()
{
    getInput("approach_radius", approachRadius);

    auto inputNodeRes = getInput<std::shared_ptr<graph_node_msgs::msg::GraphNode>>("graph_node");
    if (!inputNodeRes || !inputNodeRes.value())
    {
        RCLCPP_WARN(node->get_logger(),
                    RED "[%s] No input graph_node provided." RESET,
                    name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    auto target = *inputNodeRes.value();
    graph_node_msgs::msg::GraphNode reachable = target;

    if (!findReachablePoint(target, reachable))
    {
        RCLCPP_WARN(node->get_logger(),
                    YELLOW "[%s] Using fallback: default approach radius (%.2f m)." RESET,
                    name().c_str(), approachRadius);
        geometry_msgs::msg::Pose robotPose;
        if (robot->getPose(robotPose))
        {
            double dx = target.position.x - robotPose.position.x;
            double dy = target.position.y - robotPose.position.y;
            double dist = std::hypot(dx, dy);
            if (dist > approachRadius)
            {
                dx *= approachRadius / dist;
                dy *= approachRadius / dist;
            }
            reachable.position.x = robotPose.position.x + dx;
            reachable.position.y = robotPose.position.y + dy;
        }
    }

    setOutput("reachable_graph_node", std::make_shared<graph_node_msgs::msg::GraphNode>(reachable));
    return child_node_->executeTick();
}

bool ApproachPoseAdjustor::findReachablePoint(
    const graph_node_msgs::msg::GraphNode &target,
    graph_node_msgs::msg::GraphNode &reachable)
{
    auto costmapPtr = robot->getGlobalCostmap();
    if (!costmapPtr)
        return false;

    geometry_msgs::msg::Pose robotPose;
    if (!robot->getPose(robotPose))
        return false;

    // Extract costmap metadata
    const double res = costmapPtr->metadata.resolution;
    const double originX = costmapPtr->metadata.origin.position.x;
    const double originY = costmapPtr->metadata.origin.position.y;
    const unsigned int width = costmapPtr->metadata.size_x;
    const unsigned int height = costmapPtr->metadata.size_y;

    const auto &data = costmapPtr->data;

    // Compute direction from robot → target
    const double wx = robotPose.position.x;
    const double wy = robotPose.position.y;
    const double tx = target.position.x;
    const double ty = target.position.y;

    const double dx = tx - wx;
    const double dy = ty - wy;
    const double dist = std::hypot(dx, dy);

    if (dist < 1e-3)
        return false;

    const double ux = dx / dist;
    const double uy = dy / dist;
    const double maxDist = std::min(dist, approachRadius);

    double lastFreeX = wx;
    double lastFreeY = wy;

    for (double d = 0.0; d <= maxDist; d += res)
    {
        const double cx = wx + d * ux;
        const double cy = wy + d * uy;

        int mapX = static_cast<int>((cx - originX) / res);
        int mapY = static_cast<int>((cy - originY) / res);

        if (mapX < 0 || mapY < 0 || mapX >= static_cast<int>(width) || mapY >= static_cast<int>(height))
            break;

        const int idx = mapY * width + mapX;
        if (idx < 0 || idx >= static_cast<int>(data.size()))
            break;

        uint8_t cost = data[idx];

        // nav2_costmap_2d constants: FREE_SPACE = 0, LETHAL_OBSTACLE = 254, UNKNOWN = 255
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            reachable.position.x = lastFreeX;
            reachable.position.y = lastFreeY;
            return true;
        }

        lastFreeX = cx;
        lastFreeY = cy;
    }

    reachable.position.x = lastFreeX;
    reachable.position.y = lastFreeY;
    return true;
}
