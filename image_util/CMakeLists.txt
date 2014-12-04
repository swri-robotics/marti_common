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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR})

rosbuild_add_library(${PROJECT_NAME} 
  src/motion_estimation.cpp 
  src/image_normalization.cpp 
  src/rolling_normalization.cpp 
  src/image_matching.cpp
  src/draw_util.cpp
  src/geometry_util.cpp
  src/image_warp_util.cpp)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OpenCV_LIBRARIES})

rosbuild_add_library(${PROJECT_NAME}_nodelets
  src/nodelets/rotate_image_nodelet.cpp
  src/nodelets/contrast_stretch_nodelet.cpp
  src/nodelets/scale_image_nodelet.cpp
  src/nodelets/draw_text_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME})

# NODES
rosbuild_add_executable(normalization_image_generator_node src/nodes/normalization_image_generator_node.cpp)
target_link_libraries(normalization_image_generator_node ${PROJECT_NAME})

rosbuild_add_executable(dummy_image_publisher src/nodes/dummy_image_publisher.cpp)

rosbuild_add_executable(rotate_image src/nodes/rotate_image.cpp)

rosbuild_add_executable(scale_image src/nodes/scale_image.cpp)

rosbuild_add_executable(draw_text src/nodes/draw_text.cpp)

rosbuild_add_executable(contrast_stretch src/nodes/contrast_stretch.cpp)

# TESTS
rosbuild_add_executable(test_geometry_util test/test_geometry_util.cpp)
rosbuild_add_gtest_build_flags(test_geometry_util)
target_link_libraries(test_geometry_util ${PROJECT_NAME})

rosbuild_add_executable(test_motion_estimation test/test_motion_estimation.cpp)
rosbuild_add_gtest_build_flags(test_motion_estimation)
target_link_libraries(test_motion_estimation ${PROJECT_NAME})

rosbuild_add_executable(image_warp_tests test/image_warp_tests.cpp)
rosbuild_add_gtest_build_flags(image_warp_tests)
target_link_libraries(image_warp_tests ${PROJECT_NAME})
rosbuild_link_boost(image_warp_tests filesystem system)

rosbuild_add_executable(image_normalization_tests test/image_normalization_tests.cpp)
rosbuild_add_gtest_build_flags(image_normalization_tests)
target_link_libraries(image_normalization_tests ${PROJECT_NAME})
rosbuild_link_boost(image_normalization_tests filesystem system)

rosbuild_add_rostest(launch/geometry_util.test)
rosbuild_add_rostest(launch/motion_estimation.test)

#rosbuild_add_rostest(launch/image_util.test)
