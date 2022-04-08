cmake_minimum_required(VERSION 3.0.2)
project(swri_transform_util)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geographic_msgs
  geometry_msgs
  gps_common
  marti_nav_msgs
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

find_package(OpenCV  REQUIRED calib3d core highgui imgproc video)
find_package(Boost REQUIRED COMPONENTS filesystem serialization thread)

# First try pkg-config. Should always work for PROJ >= 8. Should work with older
# versions if they were installed with autotools
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ proj)

# Next, try CMake, which should work if PROJ was installed with CMake
if(NOT "${PROJ_FOUND}")
  unset(PROJ_FOUND CACHE)
  find_package(PROJ)
endif()

# Lastly, try to find the library directly, for backwards compatibility
if(NOT "${PROJ_FOUND}")
  set(PROJ_INCLUDE_DIRS /usr/include)
  set(PROJ_LIBRARY_DIRS /usr/lib)
  find_library(PROJ_LIBRARIES proj
    HINTS ${PROJ_LIB_DIRS}
  )
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    diagnostic_msgs
    geographic_msgs
    geometry_msgs
    gps_common
    marti_nav_msgs
    nodelet
    roscpp
    rospy
    swri_math_util
    swri_nodelet
    swri_roscpp
    swri_yaml_util
    tf
    topic_tools
  DEPENDS Boost PROJ
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJ_INCLUDE_DIRS}
)

function(set_cxx_props targ)
  if ($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set_target_properties(${targ} PROPERTIES
      CXX_STANDARD 11
      CXX_STANDARD_REQUIRED ON)
  endif()
endfunction()

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
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJ_LIBRARIES}
  Boost::filesystem
  Boost::serialization
  Boost::thread
  opencv_calib3d
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_video
)
set_cxx_props(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H")

add_library(${PROJECT_NAME}_nodelets
  src/nodelets/dynamic_publisher.cpp
  src/nodelets/gps_transform_publisher.cpp
  src/nodelets/object_transformer.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets ${catkin_LIBRARIES} ${PROJECT_NAME})
set_cxx_props(${PROJECT_NAME}_nodelets)
target_compile_definitions(${PROJECT_NAME}_nodelets PRIVATE "-D__STDC_FORMAT_MACROS") 

add_executable(lat_lon_tf_echo src/nodes/lat_lon_tf_echo.cpp)
target_link_libraries(lat_lon_tf_echo ${PROJECT_NAME})
set_cxx_props(lat_lon_tf_echo)

swri_nodelet_add_node(gps_transform_publisher ${PROJECT_NAME} GpsTransformPublisher)
target_link_libraries(gps_transform_publisher ${PROJECT_NAME}_nodelets ${catkin_LIBRARIES})
set_cxx_props(gps_transform_publisher)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_local_xy_util launch/local_xy_util.test test/test_local_xy_util.cpp)
  target_link_libraries(test_local_xy_util ${PROJECT_NAME})
  set_cxx_props(test_local_xy_util)

  add_rostest_gtest(test_utm_util launch/utm_util.test test/test_utm_util.cpp)
  target_link_libraries(test_utm_util ${PROJECT_NAME})
  set_cxx_props(test_utm_util)

  add_rostest_gtest(test_transform_manager launch/transform_manager.test test/test_transform_manager.cpp)
  target_link_libraries(test_transform_manager ${PROJECT_NAME})
  set_cxx_props(test_transform_manager)

  add_rostest_gtest(test_georeference launch/georeference.test test/test_georeference.cpp)
  target_link_libraries(test_georeference ${PROJECT_NAME})
  set_cxx_props(test_georeference)

  add_rostest_gtest(test_transform_util launch/transform_util.test test/test_transform_util.cpp)
  target_link_libraries(test_transform_util ${PROJECT_NAME})
  set_cxx_props(test_transform_util)

  add_rostest(test/initialize_invalid_gps.test)
  add_rostest(test/initialize_invalid_navsat.test)
  add_rostest(test/initialize_origin_auto_custom.test)
  add_rostest(test/initialize_origin_auto_gps.test)
  add_rostest(test/initialize_origin_auto_navsat.test)
  add_rostest(test/initialize_origin_manual.test)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelets
    lat_lon_tf_echo
    gps_transform_publisher
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
