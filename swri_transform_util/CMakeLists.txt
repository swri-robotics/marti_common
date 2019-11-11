cmake_minimum_required(VERSION 3.10)
project(swri_transform_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_roscpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED core highgui imgproc video)
find_package(Boost REQUIRED thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ proj)
if(NOT "${PROJ_FOUND}")
  set(PROJ_INCLUDE_DIRS /usr/include)
  set(PROJ_LIBRARY_DIRS /usr/lib)
  find_library(PROJ_LIBRARIES proj
    HINTS ${PROJ_LIB_DIRS}
  )
endif()

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${PROJ_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
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
set_property(TARGET ${PROJECT_NAME}
  PROPERTY POSITION_INDEPENDENT_CODE ON)
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}
  Boost::boost
  Boost::thread
  ${OpenCV_LIBS}
  ${PROJ_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs
  geographic_msgs
  geometry_msgs
  gps_msgs
  nodelet
  rclcpp
  swri_math_util
  swri_roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME}_nodes SHARED
  src/nodelets/dynamic_transform_publisher.cpp
  src/nodelets/gps_transform_publisher.cpp)
target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::DynamicTransformPublisher")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_transform_util::GpsTransformPublisher")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_nodes
  rcl_interfaces)

add_executable(lat_lon_tf_echo src/nodes/lat_lon_tf_echo.cpp)
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

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include
  ${YAML_CPP_INCLUDE_DIR}
)
ament_export_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${PROJ_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_package()
