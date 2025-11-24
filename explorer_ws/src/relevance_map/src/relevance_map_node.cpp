#include <rclcpp/rclcpp.hpp>
#include "relevance_map/relevance_map.hpp"

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<RelevanceMapNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
