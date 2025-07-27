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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "semantic_score.hpp"
#include "robot.hpp"

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

    private:
        //************ Subscribers ************//
        void semanticPromptCallback(const multimodal_query_msgs::msg::SemanticPrompt::SharedPtr msg);
        rclcpp::Subscription<multimodal_query_msgs::msg::SemanticPrompt>::SharedPtr semanticPromptSub;

        //************ Publishers ************//

        //************ Timers ************//
        rclcpp::TimerBase::SharedPtr timer;

        //************ Miscellaneous Functions ************//

        //************ Member Node Classes ************//
        std::unique_ptr<SemanticValueMap> semanticMap;
        std::unique_ptr<ServiceHandler> serviceHandler;
        std::unique_ptr<Robot> robot;
        //************ Member Variables ************//
        SemanticScore semScore;
        double timerFrequency = 100.0;  // Default timer frequency in ms
        std::string textPrompt = "default";  // Default text prompt
        std::string semanticPromptTopic = "/user_prompt";  // Default topic for semantic prompts
};

#endif // VALUE_MAP_H
