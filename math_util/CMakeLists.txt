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
rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

rosbuild_add_boost_directories()

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost COMPONENTS random) 
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions( -DBOOST_1_46 )
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/math_util.cpp
  src/trig_util.cpp
  src/random.cpp
  src/interpolation_1d.cpp
  )
rosbuild_link_boost(${PROJECT_NAME} random)
  
# Tests
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

rosbuild_add_executable(test_trig_util test/test_trig_util.cpp)
rosbuild_add_gtest_build_flags(test_trig_util)
target_link_libraries(test_trig_util ${PROJECT_NAME})

rosbuild_add_executable(test_math_util test/test_math_util.cpp)
rosbuild_add_gtest_build_flags(test_math_util)
target_link_libraries(test_math_util ${PROJECT_NAME})

rosbuild_add_executable(test_random test/test_random.cpp)
rosbuild_add_gtest_build_flags(test_random)
target_link_libraries(test_random ${PROJECT_NAME})

rosbuild_add_rostest(launch/trig_util.test)
rosbuild_add_rostest(launch/math_util.test)
rosbuild_add_rostest(launch/random.test)
