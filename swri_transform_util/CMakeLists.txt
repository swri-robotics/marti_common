cmake_minimum_required(VERSION 3.10)
project(swri_transform_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(marti_nav_msgs REQUIRED)
find_package(OpenCV REQUIRED calib3d core highgui imgproc video)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_roscpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# tf2_geometry_msgs 0.12.1 broke API compatibility, so check which version
# we need to use
if("${tf2_geometry_msgs_VERSION}" VERSION_GREATER "0.12.0")
  set(USE_NEW_TF2_TOMSG 1)
else()
  set(USE_NEW_TF2_TOMSG 0)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ proj)
if(NOT "${PROJ_FOUND}")
  set(PROJ_INCLUDE_DIRS /usr/include)
  set(PROJ_LIBRARY_DIRS /usr/lib)
  find_library(PROJ_LIBRARIES proj
    HINTS ${PROJ_LIB_DIRS}
  )
endif()

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

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
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  INTERFACE
    ${PROJ_INCLUDE_DIRS}
  PRIVATE
    ${swri_roscpp_TARGETS})

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    USE_NEW_TF2_TOMSG=${USE_NEW_TF2_TOMSG})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${GeographicLib_LIBRARIES}
  opencv_calib3d
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_video
  ${PROJ_LIBRARIES}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  swri_math_util::swri_math_util
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  yaml-cpp)

add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/dynamic_transform_publisher.cpp
  src/nodes/gps_transform_publisher.cpp
  src/nodes/obstacle_transformer.cpp)

target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(${PROJECT_NAME}_nodes
  SYSTEM
  INTERFACE
    ${PROJ_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME}
  PUBLIC "${TF_UTIL_DEFINITIONS}")

target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "${TF_UTIL_DEFINITIONS}")

target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::DynamicTransformPublisher")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::GpsTransformPublisher")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::ObstacleTransformer")
target_link_libraries(${PROJECT_NAME}_nodes
  ${PROJECT_NAME}
  ${geographic_msgs_TARGETS}
  ${gps_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${rclcpp_components_TARGETS}
  ${marti_nav_msgs_TARGETS}
  ${swri_roscpp_TARGETS})

add_executable(lat_lon_tf_echo src/nodes/lat_lon_tf_echo.cpp)
target_include_directories(lat_lon_tf_echo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(lat_lon_tf_echo
  SYSTEM
  INTERFACE
  ${PROJ_INCLUDE_DIRS})

target_link_libraries(lat_lon_tf_echo PUBLIC
  ${PROJECT_NAME}
  ${geographic_msgs_TARGETS}
  ${gps_msgs_TARGETS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake)

  ament_add_gtest_executable(transform_manager_test test/test_transform_manager.cpp)
  target_include_directories(transform_manager_test PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

  target_link_libraries(transform_manager_test
    ${PROJECT_NAME}
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros)

  ament_add_gtest(local_xy_util_test test/test_local_xy_util.cpp)
  target_link_libraries(local_xy_util_test
    ${PROJECT_NAME}
    rclcpp::rclcpp)

  ament_add_gtest(utm_util_test test/test_utm_util.cpp)
  target_link_libraries(utm_util_test
    ${PROJECT_NAME}
    rclcpp::rclcpp)

  ament_add_gtest(georeference_test test/test_georeference.cpp)
  target_link_libraries(georeference_test
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    tf2::tf2)

  ament_add_gtest(transform_util_test test/test_transform_util.cpp)
  target_link_libraries(transform_util_test
    ${PROJECT_NAME}
    rclcpp::rclcpp
    tf2::tf2)

  add_launch_test(${CMAKE_CURRENT_SOURCE_DIR}/launch/transform_manager.test.py)
  add_launch_test(${CMAKE_CURRENT_SOURCE_DIR}/launch/local_xy_util.test.py)
  add_launch_test(${CMAKE_CURRENT_SOURCE_DIR}/test/initialize_invalid_gps.test.py)
  add_launch_test(${CMAKE_CURRENT_SOURCE_DIR}/test/initialize_invalid_navsat.test.py)
  # The "custom" functionality appears to be gone in ROS 2.
  add_launch_test(${CMAKE_CURRENT_SOURCE_DIR}/test/initialize_origin_auto_gps.test.py)
  add_launch_test(${CMAKE_CURRENT_SOURCE_DIR}/test/initialize_origin_auto_navsat.test.py)
  add_launch_test(${CMAKE_CURRENT_SOURCE_DIR}/test/initialize_origin_manual.test.py)

  install(TARGETS
    transform_manager_test
    utm_util_test
    georeference_test
    transform_util_test
    local_xy_util_test
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

endif()

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS
  ${PROJECT_NAME}_nodes
  lat_lon_tf_echo
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

### Install Python Nodes/Scripts ###
install(PROGRAMS nodes/initialize_origin.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_python_install_package(${PROJECT_NAME})
ament_export_definitions("${TF_UTIL_DEFINITIONS}")
ament_export_dependencies(
  ament_cmake
  geometry_msgs
  OpenCV
  rclcpp
  swri_math_util
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor)
ament_package(CONFIG_EXTRAS cmake/swri_transform_util-extras.cmake)

