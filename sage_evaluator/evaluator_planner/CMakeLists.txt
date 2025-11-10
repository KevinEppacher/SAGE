cmake_minimum_required(VERSION 3.22)
project(evaluator_planner)

# Set CMake policy for modern find_package behavior
cmake_policy(SET CMP0074 NEW)

# Enable warnings for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################################################
# Find Required Packages
############################################################

# Core build system for ROS 2
find_package(ament_cmake REQUIRED)

# ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

############################################################
# Include Directories
############################################################

include_directories(

)

#####################################################################
#                        Declare Executables                        #
#####################################################################

# Main clustering node
add_executable(evaluator_planner_node
  src/evaluator_planner_node.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(evaluator_planner_node
  rclcpp
  nav2_costmap_2d
  rclcpp_lifecycle
  nav2_core
  pluginlib
  nav2_msgs
  tf2_ros
  geometry_msgs
  nav_msgs
)

# Use C++17 standard
target_compile_features(evaluator_planner_node PUBLIC cxx_std_17)

#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS evaluator_planner_node
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
  src
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
