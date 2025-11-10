cmake_minimum_required(VERSION 3.22)
project(evaluator_costmap)

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
find_package(rclcpp_lifecycle REQUIRED)

############################################################
# Include Directories
############################################################

include_directories(

)

#####################################################################
#                        Declare Executables                        #
#####################################################################

# Main clustering node
add_executable(evaluator_costmap_node
  src/evaluator_costmap_node.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(evaluator_costmap_node
  rclcpp
  sensor_msgs
  nav2_costmap_2d
  rclcpp_lifecycle
)

# Use C++17 standard
target_compile_features(evaluator_costmap_node PUBLIC cxx_std_17)

#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS evaluator_costmap_node
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
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
