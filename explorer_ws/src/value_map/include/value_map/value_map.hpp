#ifndef VALUE_MAP_H
#define VALUE_MAP_H

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "color.hpp"
#include "semantic_value_map.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <service_handler.hpp>
#include "std_msgs/msg/header.hpp"
#include "semantic_score.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ValueMap : public LifecycleNode {
    public:
        explicit ValueMap(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        ~ValueMap() override;

    protected:
        // Lifecycle Callbacks
        CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

        //************ Timers ************//
        void timerCallback();

        //************ Subscribers ************//
        void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:
        //************ Subscribers ************//
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbSub;
        sensor_msgs::msg::Image::SharedPtr rgbImage;
        
        //************ Publishers ************//
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub;

        //************ Timers ************//
        rclcpp::TimerBase::SharedPtr timer;
        
        //************ TF Buffer ************//
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        std::shared_ptr<rclcpp::Node> node_wrapper;
        
        //************ Member Variables ************//
        std::unique_ptr<SemanticValueMap> semanticMap;
        std::unique_ptr<ServiceHandler> serviceHandler;
        SemanticScore semScore;
        rclcpp::Time lastImageStamp;
};

#endif // VALUE_MAP_H
