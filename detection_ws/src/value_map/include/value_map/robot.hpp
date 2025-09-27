#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Robot {
    public:
        Robot(rclcpp_lifecycle::LifecycleNode* node);
        ~Robot();
        geometry_msgs::msg::PoseStamped::SharedPtr getPose(rclcpp::Time time = rclcpp::Time(0)) const;
        sensor_msgs::msg::Image::SharedPtr getImage() const;

        //************ Lifecycle Callbacks ************//
        bool on_activate();
        bool on_deactivate();
        bool on_configure();
        bool on_cleanup();
        bool on_shutdown();

        //************ Subscribers ************//
        void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);


    private:
        //************ Publishers ************//
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub;
        
        //************ Subscribers ************//
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbSub;
        sensor_msgs::msg::Image::SharedPtr rgbImage;

        //************ TF Buffer ************//
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        std::shared_ptr<rclcpp::Node> node_wrapper;

        // rclcpp::Time lastImageStamp;
        rclcpp_lifecycle::LifecycleNode* node;

        std::string mapFrame = "map";  // Default parent frame for the robot
        std::string cameraFrame = "camera";  // Default child frame for the robot
        std::string rgbTopic = "/rgb";  // Default topic for RGB images

};

#endif // ROBOT_HPP