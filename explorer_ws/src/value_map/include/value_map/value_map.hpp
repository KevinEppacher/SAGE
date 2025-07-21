#ifndef VALUE_MAP_H
#define VALUE_MAP_H

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "color.hpp"
#include "semantic_value_map.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

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

        //************ Timers ************//
        rclcpp::TimerBase::SharedPtr timer;

        //************ Member Variables ************//
        std::unique_ptr<SemanticValueMap> semanticMap;
};

#endif // VALUE_MAP_H
