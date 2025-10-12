#include "sage_behaviour_tree/robot.hpp"
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

Robot::Robot(rclcpp::Node::SharedPtr nodePtr)
: nodePtr(std::move(nodePtr))
{
    // Buffer constructed with node clock so TimePointZero returns latest transform
    tfBuffer = std::make_shared<tf2_ros::Buffer>(this->nodePtr->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
}

Robot::~Robot() = default;

bool Robot::getPose(geometry_msgs::msg::Pose & out_pose,
                        const std::string & map_frame,
                        const std::string & robot_frame)
{
    try {
        // Request latest available transform
        geometry_msgs::msg::TransformStamped tf =
            tfBuffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);

        out_pose.position.x = tf.transform.translation.x;
        out_pose.position.y = tf.transform.translation.y;
        out_pose.position.z = tf.transform.translation.z;
        out_pose.orientation = tf.transform.rotation;
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(nodePtr->get_logger(), *nodePtr->get_clock(),
                             2000, "Robot::getPose() TF lookup failed: %s", ex.what());
        return false;
    }
}