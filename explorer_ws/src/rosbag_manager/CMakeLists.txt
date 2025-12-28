cmake_minimum_required(VERSION 3.22)
project(rosbag_manager)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Ensure RPATH is set correctly for installed binaries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

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
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

############################################################
# Include Directories
############################################################

include_directories(
    include
)

#####################################################################
#                        Declare Executables                        #
#####################################################################

add_executable(recorder
    src/recorder_node.cpp
)

# Link ROS 2 dependencies
ament_target_dependencies(recorder
    rclcpp
    tf2_msgs
    geometry_msgs
    sensor_msgs
    rosbag2_storage
    rosbag2_cpp
    std_msgs
    nav_msgs
)

# Use C++17 standard
target_compile_features(recorder PUBLIC cxx_std_17)


#####################################################################
#                           Installation                            #
#####################################################################

# Install the node binary into the ROS 2 lib directory
install(TARGETS
  recorder
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
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
