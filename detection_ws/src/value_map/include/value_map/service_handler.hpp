#ifndef SERVICE_HANDLER_HPP
#define SERVICE_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "sensor_msgs/msg/image.hpp"
// #include "seem_ros_interfaces/srv/panoptic.hpp"
// #include "seem_ros_interfaces/srv/object_segmentation.hpp"
// #include "seem_ros_interfaces/srv/semantic_similarity.hpp"
#include "vlm_interface/srv/semantic_similarity.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ServiceHandler {
    public:
        ServiceHandler(rclcpp_lifecycle::LifecycleNode* node);
        ~ServiceHandler();
        bool on_configure();
        bool on_activate();
        bool on_deactivate();
        bool on_cleanup();
        bool on_shutdown();
        // sensor_msgs::msg::Image getPanopticSegmentedImage(const sensor_msgs::msg::Image &image_msg);
        // sensor_msgs::msg::Image getObjectSegmentedImage(const sensor_msgs::msg::Image &image_msg, const std::string &query);
        double getSemanticSimilarityScore(const sensor_msgs::msg::Image &image_msg, const std::string &query);

        private:
        //************ Subscribers ************//
        
        //************ Publishers ************//
        
        //************ Timers ************//
        
        //************ Services ************//
        // void callPanoptic(const sensor_msgs::msg::Image &image_msg);
        // void callObjectSegmentation(const sensor_msgs::msg::Image &image_msg, const std::string &query);
        void callSemanticSimilarity(const sensor_msgs::msg::Image &image_msg, const std::string &query);

        // rclcpp::Client<seem_ros_interfaces::srv::Panoptic>::SharedPtr cliPanoptic;
        // rclcpp::Client<seem_ros_interfaces::srv::ObjectSegmentation>::SharedPtr cliObjSeg;
        // rclcpp::Client<seem_ros_interfaces::srv::SemanticSimilarity>::SharedPtr cliSemSim;

        rclcpp::Client<vlm_interface::srv::SemanticSimilarity>::SharedPtr cliSemSim;

        //************ Member Variables ************//
        
        rclcpp_lifecycle::LifecycleNode* node;
        std::string vlmNamespace;

        sensor_msgs::msg::Image panopticSegmentedImage;
        sensor_msgs::msg::Image objectSegmentedImage;
        double semanticSimilarityScore;
};

#endif // SERVICE_HANDLER_HPP

