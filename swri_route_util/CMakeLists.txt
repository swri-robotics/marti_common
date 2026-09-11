cmake_minimum_required(VERSION 3.10)
project(swri_route_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs)
find_package(marti_common_msgs REQUIRED)
find_package(marti_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swri_geometry_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

### Build Library ###
add_library(${PROJECT_NAME} SHARED
  src/route.cpp
  src/route_point.cpp
  src/route_speeds.cpp
  src/util.cpp
  src/visualization.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${geometry_msgs_TARGETS}
  ${marti_common_msgs_TARGETS}
  ${marti_nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${swri_math_util_TARGETS}
  ${swri_transform_util_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE ${swri_geometry_util_TARGETS})

### Build Nodes ###
add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/obstacle_marker_node.cpp)

target_include_directories(${PROJECT_NAME}_nodes
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")

# This registers the component and also generates and installs a
# standalone executable that loads it, so the node can be composed
# into a container or run directly with 'ros2 run'.
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "swri_route_util::ObstacleMarkerNode"
  EXECUTABLE obstacle_marker_node)

target_link_libraries(${PROJECT_NAME}_nodes PUBLIC
  ${PROJECT_NAME}
  ${marti_nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp)
target_link_libraries(${PROJECT_NAME}_nodes PRIVATE
  ${rclcpp_components_TARGETS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_visualization test/test_visualization.cpp)
  target_link_libraries(test_visualization ${PROJECT_NAME})
endif()

### Install Libraries and Executables ###
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

### Install Header Files ###
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake
  geometry_msgs
  marti_nav_msgs
  rclcpp_components
  std_msgs
  swri_math_util
  swri_transform_util
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
ament_package()
