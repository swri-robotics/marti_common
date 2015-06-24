cmake_minimum_required(VERSION 2.4.6)
project(image_util)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  roslib
  std_msgs
  nodelet
  cv_bridge
  tf
  cmake_modules
  geometry_msgs
  image_geometry
  nav_msgs
  camera_calibration_parsers
  math_util
  opencv_util
  image_transport)

find_package(OpenCV)
find_package(Qt4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

include(${QT_USE_FILE})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    roscpp
    rospy
    roslib
    std_msgs
    nodelet
    cv_bridge
    tf
    geometry_msgs
    image_geometry
    nav_msgs
    camera_calibration_parsers
    math_util
    opencv_util
    image_transport
)

include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/image_normalization.cpp 
  src/rolling_normalization.cpp 
  src/image_matching.cpp
  src/draw_util.cpp
  src/geometry_util.cpp
  src/image_warp_util.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(${PROJECT_NAME}_nodelets
  src/nodelets/rotate_image_nodelet.cpp
  src/nodelets/contrast_stretch_nodelet.cpp
  src/nodelets/scale_image_nodelet.cpp
  src/nodelets/draw_text_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME})

add_executable(normalization_image_generator_node src/nodes/normalization_image_generator_node.cpp)
target_link_libraries(normalization_image_generator_node ${PROJECT_NAME})

add_executable(dummy_image_publisher src/nodes/dummy_image_publisher.cpp)
target_link_libraries(dummy_image_publisher ${catkin_LIBRARIES})

add_executable(rotate_image src/nodes/rotate_image.cpp)
target_link_libraries(rotate_image ${catkin_LIBRARIES})

add_executable(scale_image src/nodes/scale_image.cpp)
target_link_libraries(scale_image ${catkin_LIBRARIES})

add_executable(draw_text src/nodes/draw_text.cpp)
target_link_libraries(draw_text ${catkin_LIBRARIES})

add_executable(contrast_stretch src/nodes/contrast_stretch.cpp)
target_link_libraries(contrast_stretch ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_geometry_util launch/geometry_util.test test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})

  add_rostest_gtest(test_motion_estimation launch/motion_estimation.test test/test_motion_estimation.cpp)
  target_link_libraries(test_motion_estimation ${PROJECT_NAME})

  add_rostest_gtest(image_warp_tests launch/warp_util.test test/image_warp_tests.cpp)
  target_link_libraries(image_warp_tests ${PROJECT_NAME} ${Boost_LIBRARIES})

  add_rostest_gtest(image_normalization_tests launch/image_normalization.test test/image_normalization_tests.cpp)
  target_link_libraries(image_normalization_tests ${PROJECT_NAME} ${Boost_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} normalization_image_generator_node dummy_image_publisher rotate_image scale_image draw_text contrast_stretch
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

