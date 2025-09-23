#include <rclcpp/rclcpp.hpp>
#include "cloud_cluster.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudCluster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
