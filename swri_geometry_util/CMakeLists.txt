cmake_minimum_required(VERSION 3.10)

project(swri_geometry_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(GEOS REQUIRED)

find_package(OpenCV REQUIRED core)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cubic_spline.cpp
  src/geometry_util.cpp
  src/intersection.cpp
  src/polygon.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GEOS_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  geos_c
  Eigen3::Eigen
  opencv_core
)
ament_target_dependencies(${PROJECT_NAME}
  "cv_bridge"
  "rclcpp"
  "tf2"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_intersection test/test_intersection.cpp)
  target_link_libraries(test_intersection ${PROJECT_NAME})

  ament_add_gtest(test_geometry_util test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(tf2)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} geos_c) 

ament_package()
