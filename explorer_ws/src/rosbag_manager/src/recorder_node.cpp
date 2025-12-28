#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include <memory>

class FilteredRecorderNode : public rclcpp::Node
{
public:
    FilteredRecorderNode()
    : Node("filtered_recorder_node")
    {
        setup_writer();
        setup_subscriptions();
        RCLCPP_INFO(get_logger(), "Filtered recorder node started");
    }

private:
    rosbag2_cpp::Writer writer;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr map_metadata_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_events_sub;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub;

    template<typename MsgT>
    void create_topic(const std::string & name)
    {
        rosbag2_storage::TopicMetadata meta;
        meta.name = name;
        meta.type = rosidl_generator_traits::name<MsgT>();
        meta.serialization_format = "cdr";
        writer.create_topic(meta);
    }

    void setup_writer()
    {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = "filtered_recording";
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        writer.open(storage_options, converter_options);

        create_topic<sensor_msgs::msg::Image>("/rgb");
        create_topic<sensor_msgs::msg::Image>("/depth");
        create_topic<sensor_msgs::msg::CameraInfo>("/camera_info");

        create_topic<nav_msgs::msg::OccupancyGrid>("/map");
        create_topic<nav_msgs::msg::MapMetaData>("/map_metadata");

        create_topic<nav_msgs::msg::Odometry>("/odom");
        create_topic<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose");

        create_topic<tf2_msgs::msg::TFMessage>("/tf");
        create_topic<tf2_msgs::msg::TFMessage>("/tf_static");

        create_topic<rosgraph_msgs::msg::Clock>("/clock");
        create_topic<rcl_interfaces::msg::ParameterEvent>("/parameter_events");
        create_topic<rcl_interfaces::msg::Log>("/rosout");
    }

    void setup_subscriptions()
    {
        auto qos_sensor  = rclcpp::SensorDataQoS();
        auto qos_default = rclcpp::QoS(10);
        auto qos_latched = rclcpp::QoS(1).reliable().transient_local();

        rgb_sub = create_subscription<sensor_msgs::msg::Image>(
            "/rgb", qos_sensor,
            [this](sensor_msgs::msg::Image::SharedPtr msg)
            {
                writer.write(*msg, "/rgb", msg->header.stamp);
            });

        depth_sub = create_subscription<sensor_msgs::msg::Image>(
            "/depth", qos_sensor,
            [this](sensor_msgs::msg::Image::SharedPtr msg)
            {
                writer.write(*msg, "/depth", msg->header.stamp);
            });

        camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", qos_sensor,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                writer.write(*msg, "/camera_info", msg->header.stamp);
            });

        map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos_latched,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg)
            {
                writer.write(*msg, "/map", msg->header.stamp);
            });

        map_metadata_sub = create_subscription<nav_msgs::msg::MapMetaData>(
            "/map_metadata", qos_latched,
            [this](nav_msgs::msg::MapMetaData::SharedPtr msg)
            {
                writer.write(*msg, "/map_metadata", now());
            });

        odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_default,
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                writer.write(*msg, "/odom", msg->header.stamp);
            });

        pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", qos_latched,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
            {
                writer.write(*msg, "/pose", msg->header.stamp);
            });

        tf_sub = create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", qos_default,
            [this](tf2_msgs::msg::TFMessage::SharedPtr msg)
            {
                writer.write(*msg, "/tf", now());
            });

        tf_static_sub = create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", qos_latched,
            [this](tf2_msgs::msg::TFMessage::SharedPtr msg)
            {
                writer.write(*msg, "/tf_static", now());
            });

        clock_sub = create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", qos_default,
            [this](rosgraph_msgs::msg::Clock::SharedPtr msg)
            {
                writer.write(*msg, "/clock", msg->clock);
            });

        parameter_events_sub = create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", qos_default,
            [this](rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
            {
                writer.write(*msg, "/parameter_events", now());
            });

        rosout_sub = create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", qos_default,
            [this](rcl_interfaces::msg::Log::SharedPtr msg)
            {
                writer.write(*msg, "/rosout", now());
            });
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilteredRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
