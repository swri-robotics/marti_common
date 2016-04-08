cmake_minimum_required(VERSION 2.8.3)
project(swri_transform_util)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  tf
  pluginlib
  topic_tools
  geographic_msgs
  gps_common
  diagnostic_msgs
  swri_math_util
  swri_yaml_util
) 

find_package(OpenCV 2 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy
    tf
    pluginlib
    topic_tools
    geographic_msgs
    gps_common
    diagnostic_msgs
    swri_math_util
    swri_yaml_util
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/georeference.cpp
  src/local_xy_util.cpp
  src/utm_util.cpp
  src/transform.cpp
  src/transformer.cpp
  src/transform_manager.cpp
  src/transform_util.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} proj)

add_library(transformer_plugins
  src/utm_transformer.cpp
  src/wgs84_transformer.cpp)
target_link_libraries(transformer_plugins ${PROJECT_NAME})

add_executable(lat_lon_tf_echo src/nodes/lat_lon_tf_echo.cpp)
target_link_libraries(lat_lon_tf_echo ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_local_xy_util launch/local_xy_util.test test/test_local_xy_util.cpp)
  target_link_libraries(test_local_xy_util ${PROJECT_NAME})

  add_rostest_gtest(test_utm_util launch/utm_util.test test/test_utm_util.cpp)
  target_link_libraries(test_utm_util ${PROJECT_NAME})

  add_rostest_gtest(test_transform_manager launch/transform_manager.test test/test_transform_manager.cpp)
  target_link_libraries(test_transform_manager ${PROJECT_NAME})

  add_rostest_gtest(test_georeference launch/georeference.test test/test_georeference.cpp)
  target_link_libraries(test_georeference ${PROJECT_NAME})

  add_rostest_gtest(test_transform_util launch/transform_util.test test/test_transform_util.cpp)
  target_link_libraries(test_transform_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} lat_lon_tf_echo transformer_plugins
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

### Install Python Nodes/Scripts ###
catkin_install_python(PROGRAMS nodes/initialize_origin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES transformer_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
