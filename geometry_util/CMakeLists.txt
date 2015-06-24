cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
find_package(GEOS REQUIRED)
find_package(OpenCV)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

include_directories(include ${OpenCV_INCLUDE_DIRS} ${GEOS_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/intersection.cpp
  src/polygon.cpp
  src/geometry_util.cpp
  src/cubic_spline.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${GEOS_LIBRARIES})
