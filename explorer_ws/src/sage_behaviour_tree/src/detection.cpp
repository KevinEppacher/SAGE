#include "sage_behaviour_tree/detection.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

using namespace std::chrono_literals;

// ============================ IsDetected ============================ //

IsDetected::IsDetected(const std::string& name,
                       const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr node_ptr)
    : BT::ConditionNode(name, config), node_ptr_(std::move(node_ptr))
{
    std::string topic;
    getInput<std::string>("detection_graph_node_topic", topic);
    sub_ = node_ptr_->create_subscription<graph_node_msgs::msg::GraphNodeArray>(
        topic, 10,
        [this](const graph_node_msgs::msg::GraphNodeArray::SharedPtr msg) {
            latest_msg_ = msg;
            received_message_ = true;
            missed_ticks_ = 0;
        });
}

BT::PortsList IsDetected::providedPorts()
{
    return {
        BT::InputPort<double>(
            "detection_threshold", 0.8, "Detection threshold"),
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
    if (!received_message_) 
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] No message received yet on the topic %s. Returning RUNNING", name().c_str(),
                    sub_->get_topic_name());
        return BT::NodeStatus::RUNNING;
    }

    double threshold = 0.8;
    getInput("detection_threshold", threshold);

    double max_score = -1.0;
    std::shared_ptr<graph_node_msgs::msg::GraphNode> best_node = nullptr;
    for (auto &n : latest_msg_->nodes) 
    {
        if (n.score > max_score) 
        {
            max_score = n.score;
            best_node = std::make_shared<graph_node_msgs::msg::GraphNode>(n);
        }
    }

    if (!best_node) 
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] No nodes in the latest message. Returning FAILURE", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    setOutput("graph_nodes", best_node);

    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Best node ID: %d with score: %.2f. Returning SUCCESS", name().c_str(), best_node->id, best_node->score);


    if(max_score >= threshold) {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Detection threshold met: %.2f >= %.2f", name().c_str(), max_score, threshold);
    } else {
        RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Detection threshold not met: %.2f < %.2f", name().c_str(), max_score, threshold);
    }

    return (max_score >= threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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

  // --- Match publisher QoS (e.g. Best Effort for image topics) ---
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos.best_effort().durability_volatile();

  sub = node->create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      std::bind(&SaveImageAction::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(),
              "[%s] Subscribed to %s (QoS: BestEffort) — waiting for image...",
              name().c_str(), topic.c_str());

  startTime = node->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SaveImageAction::onRunning()
{
  if (done)
    return BT::NodeStatus::SUCCESS;

  if ((node->now() - startTime).seconds() > 10.0)
  {
    RCLCPP_WARN(node->get_logger(),
                "[%s] Timeout after 10s waiting for image.", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SaveImageAction::onHalted()
{
  sub.reset();
  RCLCPP_INFO(node->get_logger(), "[%s] Halted.", name().c_str());
}

void SaveImageAction::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (cv::imwrite(savePath, img))
    {
      RCLCPP_INFO(node->get_logger(),
                  "[%s] Image saved to %s",
                  name().c_str(), savePath.c_str());
      done = true;
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(),
                   "[%s] Failed to save image to %s",
                   name().c_str(), savePath.c_str());
    }
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "[%s] cv_bridge exception: %s",
                 name().c_str(), e.what());
  }
}
