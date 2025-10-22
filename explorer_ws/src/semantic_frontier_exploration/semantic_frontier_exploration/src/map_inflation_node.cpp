#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

class MapInflationNode : public rclcpp::Node
{
public:
    MapInflationNode() : Node("map_inflation_node")
    {
        // ---- Declare Parameters ----
        declare_parameter("map_topic", "/map");
        declare_parameter("inflated_map_topic", "/inflated_map");
        declare_parameter("inflation_radius", 0.25);           // meters
        declare_parameter("inflation_iterations", 1);          // dilation passes
        declare_parameter("kernel_type", "rect");              // rect, ellipse, cross
        declare_parameter("input_qos", "reliable");            // reliable, best_effort
        declare_parameter("output_qos", "reliable");           // reliable, best_effort

        // ---- Read Parameters ----
        loadParameters();

        // ---- Setup ----
        setupQoS();
        setupCommunication();

        // ---- Dynamic Parameter Callback ----
        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&MapInflationNode::onParameterChange, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "MapInflationNode active\n  input: %s\n  output: %s\n  radius: %.2fm\n  kernel: %s",
                    map_topic_.c_str(), inflated_topic_.c_str(), inflation_radius_, kernel_type_.c_str());
    }

private:
    //==== ROS interfaces ====//
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::QoS input_qos_{1};
    rclcpp::QoS output_qos_{1};

    //==== Parameters ====//
    std::string map_topic_;
    std::string inflated_topic_;
    std::string kernel_type_;
    std::string input_qos_str_;
    std::string output_qos_str_;
    double inflation_radius_;
    int inflation_iterations_;

    //========================= Parameter Handling =========================//
    void loadParameters()
    {
        map_topic_ = get_parameter("map_topic").as_string();
        inflated_topic_ = get_parameter("inflated_map_topic").as_string();
        inflation_radius_ = get_parameter("inflation_radius").as_double();
        inflation_iterations_ = get_parameter("inflation_iterations").as_int();
        kernel_type_ = get_parameter("kernel_type").as_string();
        input_qos_str_ = get_parameter("input_qos").as_string();
        output_qos_str_ = get_parameter("output_qos").as_string();
    }

    void setupQoS()
    {
        input_qos_ = (input_qos_str_ == "best_effort")
                         ? rclcpp::QoS(10).best_effort()
                         : rclcpp::QoS(10).reliable();

        output_qos_ = (output_qos_str_ == "best_effort")
                          ? rclcpp::QoS(1).best_effort().transient_local()
                          : rclcpp::QoS(1).reliable().transient_local();
    }

    void setupCommunication()
    {
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, input_qos_,
            std::bind(&MapInflationNode::mapCallback, this, std::placeholders::_1));

        inflated_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            inflated_topic_, output_qos_);
    }

    //========================= Dynamic Reconfigure =========================//
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool reconfigure_needed = false;

        for (const auto &p : params)
        {
            if (p.get_name() == "inflation_radius")
            {
                inflation_radius_ = p.as_double();
                RCLCPP_INFO(get_logger(), "Updated inflation_radius -> %.3fm", inflation_radius_);
            }
            else if (p.get_name() == "inflation_iterations")
            {
                inflation_iterations_ = p.as_int();
                RCLCPP_INFO(get_logger(), "Updated inflation_iterations -> %d", inflation_iterations_);
            }
            else if (p.get_name() == "kernel_type")
            {
                kernel_type_ = p.as_string();
                RCLCPP_INFO(get_logger(), "Updated kernel_type -> %s", kernel_type_.c_str());
            }
            else if (p.get_name() == "map_topic" || p.get_name() == "inflated_map_topic" ||
                     p.get_name() == "input_qos" || p.get_name() == "output_qos")
            {
                reconfigure_needed = true;
            }
        }

        if (reconfigure_needed)
        {
            loadParameters();
            setupQoS();
            setupCommunication();
            RCLCPP_INFO(get_logger(), "Reconfigured communication setup.");
        }

        return result;
    }

    //========================= Main Logic =========================//
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->data.empty())
            return;

        int w = msg->info.width;
        int h = msg->info.height;
        cv::Mat map_cv(h, w, CV_8S, (void *)msg->data.data());
        cv::Mat occ = (map_cv == 100);
        occ.convertTo(occ, CV_8U);

        // Kernel shape
        int pixels = std::max(1, (int)std::round(inflation_radius_ / msg->info.resolution));
        if (pixels % 2 == 0)
            pixels++;
        int shape = cv::MORPH_RECT;
        if (kernel_type_ == "ellipse")
            shape = cv::MORPH_ELLIPSE;
        else if (kernel_type_ == "cross")
            shape = cv::MORPH_CROSS;

        cv::Mat kernel = cv::getStructuringElement(shape, cv::Size(pixels, pixels));

        cv::Mat inflated;
        cv::dilate(occ, inflated, kernel, cv::Point(-1, -1), inflation_iterations_);

        // Build output map
        auto out = *msg;
        out.header.stamp = now();
        out.data.assign(w * h, -1);

        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                int idx = y * w + x;
                if (inflated.at<uint8_t>(y, x) > 0)
                    out.data[idx] = 100;
                else if (map_cv.at<int8_t>(y, x) == 0)
                    out.data[idx] = 0;
            }
        }

        inflated_pub_->publish(out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapInflationNode>());
    rclcpp::shutdown();
    return 0;
}
