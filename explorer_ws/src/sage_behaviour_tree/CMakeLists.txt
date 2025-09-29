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
find_package(behaviortree_cpp_v3 REQUIRED)

############################################################
# Include Directories
############################################################

include_directories(

)

#####################################################################
#                        Declare Executables                        #
#####################################################################

# Main clustering node
add_executable(bt_demo_node
  src/bt_demo_node.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(bt_demo_node
  behaviortree_cpp_v3
  rclcpp
)

# Use C++17 standard
target_compile_features(bt_demo_node PUBLIC cxx_std_17)

#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS
  bt_demo_node
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
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
