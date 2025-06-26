cmake_minimum_required(VERSION 3.8)
project(frontier_exploration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(frontier_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/SearchAlgorithms.cpp
  src/ExploreFrontierAction.cpp
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  frontier_msgs
  behaviortree_cpp
  nav2_msgs
)

add_executable(frontier_exploration_node src/FrontierExplorationNode.cpp src/SearchAlgorithms.cpp)
target_link_libraries(frontier_exploration_node ${PROJECT_NAME}_lib)
ament_target_dependencies(frontier_exploration_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  frontier_msgs
  behaviortree_cpp
  nav2_msgs
)

add_executable(frontier_discovery_node src/FrontierDiscoveryNode.cpp src/SearchAlgorithms.cpp)
target_link_libraries(frontier_discovery_node ${PROJECT_NAME}_lib)
ament_target_dependencies(frontier_discovery_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  frontier_msgs
  behaviortree_cpp
  nav2_msgs
)

add_executable(occupancy_grid_filter_node src/OccupancyGridFilters.cpp)
target_link_libraries(occupancy_grid_filter_node ${PROJECT_NAME}_lib)
ament_target_dependencies(occupancy_grid_filter_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  frontier_msgs
  nav2_msgs
)

install(TARGETS frontier_exploration_node occupancy_grid_filter_node frontier_discovery_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
