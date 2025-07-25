#include "robot.hpp"

Robot::Robot(rclcpp_lifecycle::LifecycleNode* node) : node(node)
{

}

Robot::~Robot()
{

}

bool Robot::on_configure()
{
    RCLCPP_INFO(node->get_logger(), "Configuring Robot...");
    // Configuration logic here

    rgbSub = node->create_subscription<sensor_msgs::msg::Image>(
    "/rgb", 10,
    std::bind(&Robot::rgbCallback, this, std::placeholders::_1)
    );

    node_wrapper = std::make_shared<rclcpp::Node>(
        "value_map_tf_node", 
        rclcpp::NodeOptions().start_parameter_services(false)
    );

    tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, node_wrapper, true);

    RCLCPP_INFO(node->get_logger(), "Robot configured successfully.");
    return true;
}

bool Robot::on_activate()
{
    RCLCPP_INFO(node->get_logger(), "Activating Robot...");

    posePub = node->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);

    RCLCPP_INFO(node->get_logger(), "Robot activated successfully.");
    return true;
}

bool Robot::on_deactivate()
{
    RCLCPP_INFO(node->get_logger(), "Deactivating Robot...");
    posePub.reset();
    rgbSub.reset();
    tfBuffer.reset();
    tfListener.reset();
    node_wrapper.reset();

    RCLCPP_INFO(node->get_logger(), "Robot deactivated successfully.");
    return true;
}

bool Robot::on_cleanup()
{
    RCLCPP_INFO(node->get_logger(), "Cleaning up Robot...");
    posePub.reset();
    rgbSub.reset();
    tfBuffer.reset();
    tfListener.reset();
    node_wrapper.reset();
    
    RCLCPP_INFO(node->get_logger(), "Robot cleaned up successfully.");
    return true;
}

bool Robot::on_shutdown()
{
    RCLCPP_INFO(node->get_logger(), "Shutting down Robot...");
    
    
    RCLCPP_INFO(node->get_logger(), "Robot shut down successfully.");
    return true;
}

void Robot::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    rgbImage = msg;
    lastImageStamp = msg->header.stamp;
}

geometry_msgs::msg::TransformStamped Robot::getPose()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer->lookupTransform("map", "camera", lastImageStamp);

        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header = transformStamped.header;
        poseMsg.pose.position.x = transformStamped.transform.translation.x;
        poseMsg.pose.position.y = transformStamped.transform.translation.y;
        poseMsg.pose.position.z = transformStamped.transform.translation.z;
        poseMsg.pose.orientation = transformStamped.transform.rotation;

        posePub->publish(poseMsg);
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node->get_logger(), "Could not get transform: %s", ex.what());
    }
    return transformStamped;
}

sensor_msgs::msg::Image::SharedPtr Robot::getImage() const
{
    return rgbImage;
}

