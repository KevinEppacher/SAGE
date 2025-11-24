#include "relevance_map/robot.hpp"
#include <cmath>

Robot::Robot(rclcpp::Node *node)
: node(node)
{
    node->declare_parameter<std::string>("robot.frame_map", "map");
    node->declare_parameter<std::string>("robot.frame_robot", "base_link");

    node->get_parameter("robot.frame_map", frameMap);
    node->get_parameter("robot.frame_robot", frameRobot);

    tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    RCLCPP_INFO(node->get_logger(),
                "Robot initialized with frames: map='%s', robot='%s'",
                frameMap.c_str(), frameRobot.c_str());
}

geometry_msgs::msg::PoseStamped::SharedPtr Robot::getPose()
{
    try
    {
        auto tf = tfBuffer->lookupTransform(
            frameMap, frameRobot, tf2::TimePointZero);

        auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose->header = tf.header;
        pose->pose.position.x = tf.transform.translation.x;
        pose->pose.position.y = tf.transform.translation.y;
        pose->pose.position.z = tf.transform.translation.z;
        pose->pose.orientation = tf.transform.rotation;
        return pose;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node->get_logger(),
                    "TF lookup failed (map->robot): %s", ex.what());
        return nullptr;
    }
}

double Robot::computeHorizontalFov(const sensor_msgs::msg::CameraInfo &info)
{
    double width = info.width;
    double fx = info.k[0];

    if (fx <= 0.0)
    {
        RCLCPP_WARN(node->get_logger(),
                    "CameraInfo fx=0, using fallback FOV=60 deg.");
        return 60.0;
    }

    return 2.0 * std::atan(width / (2.0 * fx)) * 180.0 / M_PI;
}
