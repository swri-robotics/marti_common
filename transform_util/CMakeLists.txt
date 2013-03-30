cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)

rosbuild_add_library(${PROJECT_NAME} 
  src/georeference.cpp
  src/local_xy_util.cpp
  src/utm_util.cpp
  src/transform.cpp
  src/transformer.cpp
  src/transform_manager.cpp
  src/transform_util.cpp)
target_link_libraries(${PROJECT_NAME} yaml-cpp proj ${OpenCV_LIBRARIES})
  
rosbuild_add_library(transformer_plugins
  src/utm_transformer.cpp
  src/wgs84_transformer.cpp)
target_link_libraries(transformer_plugins ${PROJECT_NAME})

# Tests
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

rosbuild_add_executable(test_local_xy_util test/test_local_xy_util.cpp)
rosbuild_add_gtest_build_flags(test_local_xy_util)
target_link_libraries(test_local_xy_util ${PROJECT_NAME})

rosbuild_add_executable(test_utm_util test/test_utm_util.cpp)
rosbuild_add_gtest_build_flags(test_utm_util)
target_link_libraries(test_utm_util ${PROJECT_NAME})

rosbuild_add_executable(test_transform_manager test/test_transform_manager.cpp)
rosbuild_add_gtest_build_flags(test_transform_manager)
target_link_libraries(test_transform_manager ${PROJECT_NAME})

rosbuild_add_executable(test_georeference test/test_georeference.cpp)
rosbuild_add_gtest_build_flags(test_georeference)
target_link_libraries(test_georeference ${PROJECT_NAME})

rosbuild_add_rostest(launch/local_xy_util.test)
rosbuild_add_rostest(launch/utm_util.test)
rosbuild_add_rostest(launch/transform_manager.test)
rosbuild_add_rostest(launch/georeference.test)