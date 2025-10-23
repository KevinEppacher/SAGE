cmake_minimum_required(VERSION 3.22)
project(sage_behaviour_tree)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(graph_node_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(multimodal_query_msgs REQUIRED)

############################################################
# Include Directories
############################################################

include_directories(
    include
)

#####################################################################
#                        Declare Executables                        #
#####################################################################

add_executable(sage_bt_node
    src/sage_bt_node.cpp
    src/detection.cpp
    src/navigation.cpp
    src/core.cpp
    src/utils.cpp
    src/robot.cpp
    src/decorator.cpp
    src/semantic_prompt.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(sage_bt_node
    behaviortree_cpp
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    graph_node_msgs
    rclcpp_action
    nav2_msgs
    tf2
    tf2_geometry_msgs
    visualization_msgs
    cv_bridge
    OpenCV
    sensor_msgs
    std_srvs
    multimodal_query_msgs
)

# Use C++17 standard
target_compile_features(sage_bt_node PUBLIC cxx_std_17)

#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS
  sage_bt_node
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
  launch
  include
  bt_xml
  src
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
