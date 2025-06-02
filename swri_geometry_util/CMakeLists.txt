cmake_minimum_required(VERSION 3.10)

project(swri_geometry_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GEOS REQUIRED)
find_package(OpenCV REQUIRED core)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cubic_spline.cpp
  src/geometry_util.cpp
  src/intersection.cpp
  src/polygon.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  geos_c
  opencv_core
  tf2::tf2)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_intersection test/test_intersection.cpp)
  target_link_libraries(test_intersection ${PROJECT_NAME})

  ament_add_gtest(test_geometry_util test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(tf2)

ament_package()
