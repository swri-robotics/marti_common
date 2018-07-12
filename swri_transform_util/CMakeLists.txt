cmake_minimum_required(VERSION 2.8.3)
project(swri_transform_util)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  dynamic_reconfigure
  geographic_msgs
  geometry_msgs
  gps_common
  nodelet
  roscpp
  rospy
  swri_math_util
  swri_nodelet
  swri_roscpp
  swri_yaml_util
  tf
  topic_tools
)

catkin_python_setup()

find_package(OpenCV  REQUIRED core highgui imgproc video)
find_package(Boost REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ proj)
if(NOT "${PROJ_FOUND}")
  set(PROJ_INCLUDE_DIRS /usr/include)
  set(PROJ_LIBRARY_DIRS /usr/lib)
  find_library(PROJ_LIBRARIES proj
    HINTS ${PROJ_LIB_DIRS}
  )
endif()

generate_dynamic_reconfigure_options(
  cfg/DynamicPublisher.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    diagnostic_msgs
    dynamic_reconfigure
    geographic_msgs
    geometry_msgs
    gps_common
    nodelet
    roscpp
    rospy
    swri_math_util
    swri_nodelet
    swri_roscpp
    swri_yaml_util
    tf
    topic_tools
  DEPENDS PROJ
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${PROJ_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/georeference.cpp
  src/local_xy_util.cpp
  src/utm_util.cpp
  src/transform.cpp
  src/transformer.cpp
  src/transform_manager.cpp
  src/transform_util.cpp
  src/utm_transformer.cpp
  src/wgs84_transformer.cpp)
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJ_LIBRARIES})

add_library(${PROJECT_NAME}_nodelets
  src/nodelets/dynamic_publisher.cpp
  src/nodelets/gps_transform_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gencfg)

add_executable(lat_lon_tf_echo src/nodes/lat_lon_tf_echo.cpp)
target_link_libraries(lat_lon_tf_echo ${PROJECT_NAME})

swri_nodelet_add_node(gps_transform_publisher ${PROJECT_NAME} GpsTransformPublisher)
target_link_libraries(gps_transform_publisher ${PROJECT_NAME}_nodelets ${catkin_LIBRARIES})

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

  add_rostest(test/initialize_invalid_gps.test)
  add_rostest(test/initialize_invalid_navsat.test)
  add_rostest(test/initialize_origin_auto_gps.test)
  add_rostest(test/initialize_origin_auto_navsat.test)
  add_rostest(test/initialize_origin_manual.test)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelets lat_lon_tf_echo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

### Install Python Nodes/Scripts ###
catkin_install_python(PROGRAMS nodes/initialize_origin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
