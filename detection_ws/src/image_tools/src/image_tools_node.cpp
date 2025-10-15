#include <rclcpp/rclcpp.hpp>
#include "image_tools.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageTools>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}