#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class EvaluatorPlannerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

    EvaluatorPlannerNode()
        : LifecycleNode("evaluator_planner_node")
    {
        declare_parameter("costmap_topic", "/evaluator/costmap");
        declare_parameter("planner_plugin", "nav2_navfn_planner/NavfnPlanner");

        costmap_topic_ = get_parameter("costmap_topic").as_string();
        planner_plugin_name_ = get_parameter("planner_plugin").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to externally published costmap
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_,
            rclcpp::QoS(1).transient_local(),
            std::bind(&EvaluatorPlannerNode::costmapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "EvaluatorPlannerNode constructed and subscribing to %s", costmap_topic_.c_str());
    }

    // ------------------------------------------------------------------
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Waiting for costmap data...");
        while (rclcpp::ok() && !costmap_) {
            rclcpp::spin_some(get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(get_logger(), "Costmap received.");

        try {
            planner_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
                "nav2_core", "nav2_core::GlobalPlanner");
            planner_ = planner_loader_->createSharedInstance(planner_plugin_name_);

            // Wrap your static costmap into Costmap2DROS
            costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("evaluator_costmap");
            costmap_ros_->on_configure(rclcpp_lifecycle::State());
            costmap_ros_->on_activate(rclcpp_lifecycle::State());

            *(costmap_ros_->getCostmap()) = *costmap_;
            costmap_ros_->resetLayers();

            planner_->configure(shared_from_this(), "GridBased", tf_buffer_, costmap_ros_);
            planner_->activate();

            action_server_ = rclcpp_action::create_server<ComputePathToPose>(
                shared_from_this(),
                "compute_path_to_pose",
                std::bind(&EvaluatorPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&EvaluatorPlannerNode::handleCancel, this, std::placeholders::_1),
                std::bind(&EvaluatorPlannerNode::handleAccepted, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Planner configured and action server ready.");
            return nav2_util::CallbackReturn::SUCCESS;
        } catch (const std::exception &e) {
            RCLCPP_FATAL(get_logger(), "Failed to load planner: %s", e.what());
            return nav2_util::CallbackReturn::FAILURE;
        }
    }

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        if (planner_) {
            planner_->deactivate();
            planner_->cleanup();
        }
        planner_.reset();
        costmap_.reset();
        RCLCPP_INFO(get_logger(), "Planner cleaned up.");
        return nav2_util::CallbackReturn::SUCCESS;
    }

private:
    // ------------------------------------------------------------------
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!costmap_) {
            costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
                msg->info.width,
                msg->info.height,
                msg->info.resolution,
                msg->info.origin.position.x,
                msg->info.origin.position.y);
        }

        for (unsigned int y = 0; y < msg->info.height; ++y) {
            for (unsigned int x = 0; x < msg->info.width; ++x) {
                unsigned char c = static_cast<unsigned char>(msg->data[y * msg->info.width + x]);
                costmap_->setCost(x, y, c);
            }
        }
    }

    // ------------------------------------------------------------------
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ComputePathToPose::Goal>)
    {
        RCLCPP_INFO(get_logger(), "Received ComputePathToPose goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread(&EvaluatorPlannerNode::execute, this, goal_handle).detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ComputePathToPose::Result>();

        if (!planner_) {
            RCLCPP_ERROR(get_logger(), "Planner not configured yet.");
            goal_handle->abort(result);
            return;
        }

        try {
            auto path = planner_->createPlan(goal->start, goal->goal);

            // Publish path for visualization
            if (!path_pub_) {
                path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/evaluator/planned_path", 1);
            }
            path.header.frame_id = "robot_original_pose_at_scan";  // match your costmap frame
            path.header.stamp = now();
            path_pub_->publish(path);

            result->path = path;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Returned path with %zu poses", path.poses.size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Planner error: %s", e.what());
            goal_handle->abort(result);
        }
    }

    // ------------------------------------------------------------------
    std::string costmap_topic_;
    std::string planner_plugin_name_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

    std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> planner_loader_;
    std::shared_ptr<nav2_core::GlobalPlanner> planner_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

// ----------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EvaluatorPlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
