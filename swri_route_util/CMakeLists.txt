cmake_minimum_required(VERSION 3.10)
project(swri_route_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs)
find_package(marti_common_msgs REQUIRED)
find_package(marti_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
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
  ${swri_math_util_TARGETS}
  ${swri_transform_util_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE ${swri_geometry_util_TARGETS})

### Install Libraries and Executables ###
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
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
  swri_math_util
  swri_transform_util
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
ament_package()
