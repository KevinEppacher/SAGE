cmake_minimum_required(VERSION 3.22)
project(relevance_map)

# Set CMake policy for modern find_package behavior
cmake_policy(SET CMP0074 NEW)

# Enable warnings for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

############################################################
# Find Required Packages
############################################################

# Core build system for ROS 2
find_package(ament_cmake REQUIRED)

# ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(multimodal_query_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(graph_node_msgs REQUIRED)

############################################################
# Include Directories
############################################################

include_directories(
  include                           # Local headers
  include/relevance_map
)

#####################################################################
#                        Declare Executables                        #
#####################################################################

# Main clustering node
add_executable(relevance_map_node
  src/relevance_map_node.cpp
  src/relevance_map/relevance_map.cpp
  src/relevance_map/robot.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(relevance_map_node
  rclcpp
  sensor_msgs
  rclcpp_lifecycle
  cv_bridge
  tf2_ros
  nav_msgs
  OpenCV
  tf2
  tf2_geometry_msgs
  multimodal_query_msgs
  std_srvs
  graph_node_msgs
)

# Link OpenCV
target_link_libraries(relevance_map_node 
  ${OpenCV_LIBS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

# Use C++17 standard
target_compile_features(relevance_map_node PUBLIC cxx_std_17)

#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS
  relevance_map_node
  DESTINATION lib/${PROJECT_NAME}
)
#####################################################################
#                      Linting and Testing                          #
#####################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Optionally disable some linters (uncomment if needed)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

#####################################################################
#                           Finalization                            #
#####################################################################

# Install launch files
install(DIRECTORY
  include
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()