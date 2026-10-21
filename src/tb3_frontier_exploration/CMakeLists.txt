cmake_minimum_required(VERSION 3.8)
project(tb3_frontier_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Frontier detection node
add_executable(frontier_detection_node src/frontier_detection_node.cpp)
ament_target_dependencies(frontier_detection_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs)

# Goal assignment node
add_executable(goal_assignment_node src/goal_assignment_node.cpp)
ament_target_dependencies(goal_assignment_node
  rclcpp nav_msgs geometry_msgs nav2_msgs tf2 tf2_ros std_msgs)

# Install
install(TARGETS
  frontier_detection_node
  goal_assignment_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/startup_map_warmup_node.py
  DESTINATION lib/${PROJECT_NAME})

# Only params.yaml lives here now. The Gazebo worlds and the vendored models
# they reference moved to tb3_bringup, next to the launch files that load them.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
