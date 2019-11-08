cmake_minimum_required(VERSION 3.10)
project(swri_route_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(marti_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swri_geometry_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_roscpp REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

### Build Library ###
add_library(${PROJECT_NAME} 
  src/route.cpp
  src/route_point.cpp
  src/route_speeds.cpp
  src/util.cpp
  src/visualization.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
  marti_common_msgs
  marti_nav_msgs
  rclcpp
  swri_geometry_util
  swri_math_util
  swri_roscpp
  swri_transform_util
  visualization_msgs
)

### Install Libraries and Executables ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

### Install Header Files ###
install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
