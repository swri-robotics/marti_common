cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(Boost REQUIRED COMPONENTS system filesystem)

if(${Boost_MAJOR_VERSION} LESS 2 AND ${Boost_MINOR_VERSION} LESS 48)
  add_definitions( -DBOOST_FILESYSTEM_VERSION=2 )
  message ("  Using Boost Filesystem Version 2")
else(${Boost_MAJOR_VERSION} LESS 2 AND ${Boost_MINOR_VERSION} LESS 48)
  add_definitions( -DBOOST_FILESYSTEM_VERSION=3 )
  message ("  Using Boost Filesystem Version 3")
endif(${Boost_MAJOR_VERSION} LESS 2 AND ${Boost_MINOR_VERSION} LESS 48)

rosbuild_init()
rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/file_util.cpp)
rosbuild_link_boost(${PROJECT_NAME} system filesystem)
  
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

rosbuild_add_executable(test_file_util test/test_file_util.cpp)
rosbuild_add_gtest_build_flags(test_file_util)
target_link_libraries(test_file_util ${PROJECT_NAME})

rosbuild_add_rostest(launch/file_util.test)
