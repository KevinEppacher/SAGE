#include <rclcpp/rclcpp.hpp>
#include "semantic_frontier_exploration.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SemanticFrontier>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
