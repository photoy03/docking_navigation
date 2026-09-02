cmake_minimum_required(VERSION 3.8)
project(robot1_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

install(
  DIRECTORY
    launch
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)

add_executable(
  destination_resolver
  src/destination_resolver.cpp
)

add_executable(
  nav_goal_client
  src/nav_goal_client.cpp
)

add_executable(
  localization_startup_gate
  src/localization_startup_gate.cpp
)

add_executable(
  localization_ready_gate
  src/localization_ready_gate.cpp
)

add_executable(
  mode_manager
  src/mode_manager.cpp
)

ament_target_dependencies(
  mode_manager
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(
  localization_ready_gate
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(
  localization_startup_gate
  rclcpp
  std_msgs
  nav2_msgs
)

ament_target_dependencies(
  nav_goal_client
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

ament_target_dependencies(
  destination_resolver
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(
  destination_resolver
  nlohmann_json::nlohmann_json
)

install(
  TARGETS
    destination_resolver
    nav_goal_client
    localization_startup_gate
    localization_ready_gate
    mode_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
