cmake_minimum_required(VERSION 3.8)
project(uosm_robot_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz2 REQUIRED)

# Add executable
add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher
  rclcpp
  std_msgs
)

# Install executable
install(TARGETS
  state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY
    launch
    urdf
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()