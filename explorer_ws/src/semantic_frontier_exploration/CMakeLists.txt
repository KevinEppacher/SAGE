cmake_minimum_required(VERSION 3.22)
project(semantic_frontier_exploration)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(frontier_msgs REQUIRED)

# Point Cloud Library (PCL) components
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)

############################################################
# Include Directories
############################################################

include_directories(
  include                           # Local headers
  include/semantic_frontier_exploration
)

#####################################################################
#                        Declare Executables                        #
#####################################################################

# Main clustering node
add_executable(semantic_frontier_exploration_node
  src/main.cpp
  src/semantic_frontier_exploration/semantic_frontier_exploration.cpp
  src/semantic_frontier_exploration/frontier.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(semantic_frontier_exploration_node
  rclcpp
  sensor_msgs
  visualization_msgs
  std_msgs
  nav_msgs
  frontier_msgs
)

# Link OpenCV
target_link_libraries(semantic_frontier_exploration_node ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Define PCL macros (if any)
target_compile_definitions(semantic_frontier_exploration_node PUBLIC ${PCL_DEFINITIONS})

# Use C++17 standard
target_compile_features(semantic_frontier_exploration_node PUBLIC cxx_std_17)

#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS semantic_frontier_exploration_node
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
  src
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
