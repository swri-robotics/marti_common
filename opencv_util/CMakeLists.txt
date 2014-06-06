cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(OpenCV REQUIRED)

rosbuild_init()
rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

rosbuild_add_boost_directories()

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost COMPONENTS random) 
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions( -DBOOST_1_46 )
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_add_library(${PROJECT_NAME} 
  src/models.cpp
  src/model_fit.cpp
  src/show.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
