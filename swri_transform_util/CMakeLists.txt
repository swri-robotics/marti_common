cmake_minimum_required(VERSION 3.10)
project(swri_transform_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(marti_nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_roscpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# tf2_geometry_msgs 0.12.1 broke API compatibility, so check which version
# we need to use
find_package(tf2_geometry_msgs REQUIRED)
if("${tf2_geometry_msgs_VERSION}" VERSION_GREATER "0.12.0")
  set(USE_NEW_TF2_TOMSG 1)
else()
  set(USE_NEW_TF2_TOMSG 0)
endif()

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED calib3d core highgui imgproc video)
find_package(Boost REQUIRED filesystem thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ proj)
if(NOT "${PROJ_FOUND}")
  set(PROJ_INCLUDE_DIRS /usr/include)
  set(PROJ_LIBRARY_DIRS /usr/lib)
  find_library(PROJ_LIBRARIES proj
    HINTS ${PROJ_LIB_DIRS}
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/georeference.cpp
  src/local_xy_util.cpp
  src/utm_util.cpp
  src/transform.cpp
  src/transformer.cpp
  src/transform_manager.cpp
  src/transform_util.cpp
  src/tf2_util.cpp
  src/utm_transformer.cpp
  src/wgs84_transformer.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}
  SYSTEM
  INTERFACE
  ${PROJ_INCLUDE_DIRS}
)
set_property(TARGET ${PROJECT_NAME}
  PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    USE_NEW_TF2_TOMSG=${USE_NEW_TF2_TOMSG}
)

if ("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
  target_compile_definitions(${PROJECT_NAME} PUBLIC "-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H")
endif()

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Boost::boost
  Boost::filesystem
  Boost::thread
  opencv_calib3d
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_video
  ${PROJ_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  gps_msgs
  rclcpp
  sensor_msgs
  swri_math_util
  swri_roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/dynamic_transform_publisher.cpp
  src/nodes/gps_transform_publisher.cpp
  src/nodes/obstacle_transformer.cpp
)
target_include_directories(${PROJECT_NAME}_nodes
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_nodes
  SYSTEM
  INTERFACE
  ${PROJ_INCLUDE_DIRS}
)

set(TF_UTIL_DEFINITIONS "")
if ("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
  set(TF_UTIL_DEFINITIONS "-DUSE_TF2_H_FILES ${TF_UTIL_DEFINITIONS}")
endif()

if ("$ENV{ROS_DISTRO}" STRGREATER "galactic")
  set(TF_UTIL_DEFINITIONS "-DUSE_PROJ_API_6 ${TF_UTIL_DEFINITIONS}")
endif()

# Workaround to https://github.com/ros2/geometry2/pull/427
if (("$ENV{ROS_DISTRO}" STREQUAL "rolling") OR ("$ENV{ROS_DISTRO}" STREQUAL "humble"))
  set(TF_UTIL_DEFINITIONS "-DFROM_MSG_WORKAROUND ${TF_UTIL_DEFINITIONS}")
endif()

target_compile_definitions(${PROJECT_NAME}
  PUBLIC "${TF_UTIL_DEFINITIONS}"
  )
target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "${TF_UTIL_DEFINITIONS}"
  )

target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::DynamicTransformPublisher")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::GpsTransformPublisher")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::ObstacleTransformer")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_nodes
  marti_nav_msgs
  rcl_interfaces
  swri_roscpp
  rclcpp_components
)

add_executable(lat_lon_tf_echo src/nodes/lat_lon_tf_echo.cpp)
target_include_directories(lat_lon_tf_echo
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(lat_lon_tf_echo
  SYSTEM
  INTERFACE
  ${PROJ_INCLUDE_DIRS}
)

if ("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
  target_compile_definitions(lat_lon_tf_echo PRIVATE "-DUSE_TF2_H_FILES")
endif()

target_link_libraries(lat_lon_tf_echo ${PROJECT_NAME})

# TODO pjr Convert unit tests
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
# 
#  add_rostest_gtest(test_local_xy_util launch/local_xy_util.test test/test_local_xy_util.cpp)
#  target_link_libraries(test_local_xy_util ${PROJECT_NAME})
#
#  add_rostest_gtest(test_utm_util launch/utm_util.test test/test_utm_util.cpp)
#  target_link_libraries(test_utm_util ${PROJECT_NAME})
#
#  add_rostest_gtest(test_transform_manager launch/transform_manager.test test/test_transform_manager.cpp)
#  target_link_libraries(test_transform_manager ${PROJECT_NAME})
#
#  add_rostest_gtest(test_georeference launch/georeference.test test/test_georeference.cpp)
#  target_link_libraries(test_georeference ${PROJECT_NAME})
#
#  add_rostest_gtest(test_transform_util launch/transform_util.test test/test_transform_util.cpp)
#  target_link_libraries(test_transform_util ${PROJECT_NAME})
#
#  add_rostest(test/initialize_invalid_gps.test)
#  add_rostest(test/initialize_invalid_navsat.test)
#  add_rostest(test/initialize_origin_auto_custom.test)
#  add_rostest(test/initialize_origin_auto_gps.test)
#  add_rostest(test/initialize_origin_auto_navsat.test)
#  add_rostest(test/initialize_origin_manual.test)
# endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_nodes
  lat_lon_tf_echo
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

### Install Python Nodes/Scripts ###
install(PROGRAMS nodes/initialize_origin.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_export_definitions("${TF_UTIL_DEFINITIONS}")
ament_export_dependencies(
  ament_cmake
  geographic_msgs
  gps_msgs
  tf2_geometry_msgs
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  ${PROJ_LIBRARIES}
)
ament_package()

