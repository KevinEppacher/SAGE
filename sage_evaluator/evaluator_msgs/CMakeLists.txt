cmake_minimum_required(VERSION 3.10)
project(evaluator_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set CMake policies to suppress warnings (keep OLD for ROS2 Humble compatibility)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
    DIRECTORY msg
    DESTINATION share/${PROJECT_NAME}
)

# Necessary for the message generation
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multimodal_query_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/EvaluationSummary.msg"
    "msg/EvaluationEvent.msg"
    "msg/EvaluationResult.msg"
    "msg/EvaluationPathEvent.msg"
    DEPENDENCIES 
      std_msgs
      sensor_msgs
      geometry_msgs
      multimodal_query_msgs
      nav_msgs
)

ament_package()