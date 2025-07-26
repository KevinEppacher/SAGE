cmake_minimum_required(VERSION 3.22)
project(value_map)

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
find_package(seem_ros_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(frontier_msgs REQUIRED)
# find_package(graph_node_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)

# find_package(nav2_costmap_2d REQUIRED)
# find_package(rclcpp_components REQUIRED)

# Point Cloud Library (PCL) components
# find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)

############################################################
# Include Directories
############################################################

include_directories(
  include                           # Local headers
  include/value_map
)

#####################################################################
#                        Declare Executables                        #
#####################################################################

# Main clustering node
add_executable(value_map_node
  src/value_map_node.cpp
  src/value_map/value_map.cpp
  src/value_map/semantic_value_map.cpp
  src/value_map/service_handler.cpp
  src/value_map/robot.cpp
  src/value_map/color.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(value_map_node
  rclcpp
  sensor_msgs
  rclcpp_lifecycle
  cv_bridge
  seem_ros_interfaces
  tf2_ros
  nav_msgs
  OpenCV
  tf2
  tf2_geometry_msgs
  # visualization_msgs
  # std_msgs
  # frontier_msgs
  # graph_node_msgs
  # pcl_conversions
)

# Link OpenCV
target_link_libraries(value_map_node 
  ${OpenCV_LIBS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

# Use C++17 standard
target_compile_features(value_map_node PUBLIC cxx_std_17)

#####################################################################

# Main clustering node
add_executable(test_image_subscriber
  src/test/test_image_subscriber.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(test_image_subscriber
  rclcpp
  sensor_msgs
  rclcpp_lifecycle
  cv_bridge
)

# Link OpenCV
target_link_libraries(test_image_subscriber
  ${OpenCV_LIBS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

# Use C++17 standard
target_compile_features(test_image_subscriber PUBLIC cxx_std_17)

#####################################################################

# Main clustering node
add_executable(test_service
  src/test/test_service.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(test_service
  rclcpp
  sensor_msgs
  seem_ros_interfaces
  cv_bridge
  rclcpp_lifecycle
)

# Link OpenCV
target_link_libraries(test_service
  ${OpenCV_LIBS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

# Use C++17 standard
target_compile_features(test_service PUBLIC cxx_std_17)

#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS
  value_map_node
  test_image_subscriber
  test_service
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
  launch
  rviz
  # src
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()