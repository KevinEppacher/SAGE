cmake_minimum_required(VERSION 3.10)
project(graph_node_msgs)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################################################
# Find Required Packages
############################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Point Cloud Library (PCL) components
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)

############################################################
# Message generation
############################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraphNode.msg"
  "msg/GraphNodeArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

############################################################
# C++ Library
############################################################

add_library(graph_node_lib
  src/graph_node.cpp
  src/graph_node_collection.cpp
)

target_include_directories(graph_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(graph_node_lib
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  rosgraph_msgs
  pcl_conversions
)

target_link_libraries(graph_node_lib
  ${PCL_LIBRARIES}
)

# Link against generated ROS2 messages
rosidl_target_interfaces(graph_node_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

############################################################
# Installation
############################################################

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library and export config
install(TARGETS graph_node_lib
  EXPORT export_graph_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT export_graph_node_lib
  FILE export_graph_node_lib.cmake
  NAMESPACE graph_node_msgs::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Optional: for reference
install(DIRECTORY
  msg
  src
  DESTINATION share/${PROJECT_NAME}
)

############################################################
# Testing (optional)
############################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

############################################################
# Export für andere Packages
############################################################
ament_export_include_directories(include)
ament_export_targets(export_graph_node_lib)
ament_export_dependencies(rclcpp geometry_msgs std_msgs visualization_msgs pcl_conversions)

ament_package()
