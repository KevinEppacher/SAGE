#include "sage_behaviour_tree/detection.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include "sage_behaviour_tree/colors.hpp"

using namespace std::chrono_literals;

// ============================ IsDetected ============================ //

IsDetected::IsDetected(const std::string& name,
                       const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr nodePtr)
    : BT::ConditionNode(name, config),
      nodePtr(std::move(nodePtr))
{
    std::string topic;
    getInput<std::string>("detection_graph_node_topic", topic);

    subscriber = this->nodePtr->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        topic, 10,
        [this](const graph_node_msgs::msg::GraphNodeArray::SharedPtr msg)
        {
            latestMsg = msg;
            receivedMessage = true;
            missedTicks = 0;
        });

    RCLCPP_INFO(this->nodePtr->get_logger(),
                "[%s] Subscribed to detection topic: %s",
                this->name().c_str(), topic.c_str());
}

BT::PortsList IsDetected::providedPorts()
{
    return {
        BT::InputPort<double>("detection_threshold", 0.8, "Minimum detection score threshold"),
        BT::InputPort<double>("time_threshold", 1.0, "Seconds the detection must persist"),
        BT::InputPort<std::string>(
            "detection_graph_node_topic",
            "/detection_graph_nodes/graph_nodes",
            "GraphNodeArray topic"),
        BT::OutputPort<std::shared_ptr<graph_node_msgs::msg::GraphNode>>(
            "graph_nodes", "Best detected GraphNode")
    };
}

BT::NodeStatus IsDetected::tick()
{
    if (!receivedMessage)
    {
        RCLCPP_WARN_THROTTLE(nodePtr->get_logger(), *nodePtr->get_clock(), 2000,
                             ORANGE "[%s] Waiting for detection messages..." RESET,
                             name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    double threshold = 0.8;
    double timeThreshold = 1.0;
    getInput("detection_threshold", threshold);
    getInput("time_threshold", timeThreshold);

    // Find node with maximum score
    double maxScore = -1.0;
    std::shared_ptr<graph_node_msgs::msg::GraphNode> bestNode = nullptr;

    for (auto& n : latestMsg->nodes)
    {
        if (n.score > maxScore)
        {
            maxScore = n.score;
            bestNode = std::make_shared<graph_node_msgs::msg::GraphNode>(n);
        }
    }

    if (!bestNode)
    {
        RCLCPP_WARN_THROTTLE(nodePtr->get_logger(), *nodePtr->get_clock(), 2000,
                    RED "[%s] No detections in message. Returning FAILURE" RESET,
                    name().c_str());
        aboveThreshold = false;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("graph_nodes", bestNode);

    // --- Detection stability logic using steady clock ---
    rclcpp::Time now = steadyClock.now();

    if (maxScore >= threshold)
    {
        if (!aboveThreshold)
        {
            detectionStartTime = now;
            aboveThreshold = true;
            RCLCPP_INFO(nodePtr->get_logger(),
                        YELLOW "[%s] Detection above %.2f (%.2f). Timer started [steady clock]." RESET,
                        name().c_str(), threshold, maxScore);
        }

        double elapsed = (now - detectionStartTime).seconds();

        if (elapsed >= timeThreshold)
        {
            if (timeThreshold > 0.0)
            {
                RCLCPP_INFO(nodePtr->get_logger(),
                            GREEN "[%s] Detection stable for %.2fs (score %.2f ≥ %.2f) → SUCCESS" RESET,
                            name().c_str(), elapsed, maxScore, threshold);
            }
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            if (timeThreshold > 0.0)
            {
                RCLCPP_INFO_THROTTLE(nodePtr->get_logger(), *nodePtr->get_clock(), 1000,
                                    ORANGE "[%s] Detection ongoing %.2fs / %.2fs (score %.2f)" RESET,
                                    name().c_str(), elapsed, timeThreshold, maxScore);
            }
            return BT::NodeStatus::FAILURE;
        }
    }
    else
    {
        if (aboveThreshold)
        {
            if (timeThreshold > 0.0)
            {
                RCLCPP_WARN(nodePtr->get_logger(),
                            RED "[%s] Detection dropped below %.2f (%.2f < %.2f). Resetting timer." RESET,
                            name().c_str(), threshold, maxScore, threshold);
            }
        }
        aboveThreshold = false;
        return BT::NodeStatus::FAILURE;
    }
}

// ============================ SaveImageAction ============================ //

SaveImageAction::SaveImageAction(const std::string &name,
                                 const BT::NodeConfiguration &config,
                                 rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node(std::move(node))
{
}

BT::PortsList SaveImageAction::providedPorts()
{
    return {
        BT::InputPort<std::string>("imageTopic", "/yoloe/overlay", "Image topic to subscribe"),
        BT::InputPort<std::string>("savePath", "/tmp/overlay.jpg", "Output image path")};
}

BT::NodeStatus SaveImageAction::onStart()
{
    getInput("imageTopic", topic);
    getInput("savePath", savePath);

    done = false;
    sub.reset();

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort().durability_volatile();

    sub = node->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        std::bind(&SaveImageAction::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(),
                ORANGE "[%s] Subscribed to %s (QoS: BestEffort) — waiting for image..." RESET,
                name().c_str(), topic.c_str());

    startTime = node->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SaveImageAction::onRunning()
{
    if (done)
    {
        RCLCPP_INFO(node->get_logger(),
                    GREEN "[%s] Image successfully saved." RESET,
                    name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    if ((node->now() - startTime).seconds() > 10.0)
    {
        RCLCPP_WARN(node->get_logger(),
                    RED "[%s] Timeout after 10s waiting for image." RESET,
                    name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                         ORANGE "[%s] Still waiting for image..." RESET,
                         name().c_str());

    return BT::NodeStatus::RUNNING;
}

void SaveImageAction::onHalted()
{
    sub.reset();
    RCLCPP_INFO(node->get_logger(),
                YELLOW "[%s] Halted." RESET,
                name().c_str());
}

void SaveImageAction::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (done) return;

    try
    {
        // Convert ROS image to OpenCV
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // --- Ensure directory exists ---
        fs::path savePathFS(savePath);
        fs::path directory = savePathFS.parent_path();

        if (!fs::exists(directory))
        {
            try
            {
                fs::create_directories(directory);
                RCLCPP_INFO(node->get_logger(),
                            GREEN "[%s] Created directory path: %s" RESET,
                            name().c_str(), directory.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node->get_logger(),
                             RED "[%s] Failed to create directory '%s': %s" RESET,
                             name().c_str(), directory.c_str(), e.what());
                return;
            }
        }

        // --- Save image ---
        if (cv::imwrite(savePath, img))
        {
            RCLCPP_INFO(node->get_logger(),
                        GREEN "[%s] 💾 Image saved to %s" RESET,
                        name().c_str(), savePath.c_str());
            done = true;
            sub.reset();
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(),
                         RED "[%s] Failed to save image to %s" RESET,
                         name().c_str(), savePath.c_str());
        }
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     RED "[%s] cv_bridge exception: %s" RESET,
                     name().c_str(), e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(),
                     RED "[%s] Exception while saving image: %s" RESET,
                     name().c_str(), e.what());
    }
}
