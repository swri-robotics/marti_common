cmake_minimum_required(VERSION 2.8.3)
project(swri_image_util)

find_package(catkin REQUIRED COMPONENTS
  camera_calibration_parsers
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  message_filters
  nav_msgs
  nodelet
  roscpp
  rospy
  std_msgs
  swri_geometry_util
  swri_math_util
  swri_nodelet
  swri_opencv_util
  swri_roscpp
  tf
)

find_package(OpenCV REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Boost REQUIRED COMPONENTS filesystem system random)

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions( -DBOOST_1_46 )
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to pkg-config
  find_package(PkgConfig)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  DEPENDS OpenCV EIGEN3
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    image_geometry
    image_transport
    message_filters
    nav_msgs
    nodelet
    roscpp
    rospy
    std_msgs
    swri_math_util
    swri_opencv_util
    swri_roscpp
    tf
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/blend_images_util.cpp
  src/draw_util.cpp
  src/geometry_util.cpp
  src/image_matching.cpp
  src/image_normalization.cpp
  src/image_warp_util.cpp
  src/replace_colors.cpp
  src/rolling_normalization.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_nodelets
  src/nodelets/blend_images_nodelet.cpp
  src/nodelets/crosshairs_nodelet.cpp
  src/nodelets/contrast_stretch_nodelet.cpp
  src/nodelets/draw_polygon_nodelet.cpp
  src/nodelets/draw_text_nodelet.cpp
  src/nodelets/image_pub_nodelet.cpp
  src/nodelets/normalize_response_nodelet.cpp
  src/nodelets/replace_colors_nodelet.cpp
  src/nodelets/rotate_image_nodelet.cpp
  src/nodelets/scale_image_nodelet.cpp
  src/nodelets/warp_image_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME})

add_executable(blend_images src/nodes/blend_images_node.cpp)
target_link_libraries(blend_images ${catkin_LIBRARIES})

add_executable(contrast_stretch src/nodes/contrast_stretch.cpp)
target_link_libraries(contrast_stretch ${catkin_LIBRARIES})

add_executable(draw_text src/nodes/draw_text.cpp)
target_link_libraries(draw_text ${catkin_LIBRARIES})

add_executable(dummy_image_publisher src/nodes/dummy_image_publisher.cpp)
target_link_libraries(dummy_image_publisher ${catkin_LIBRARIES})

add_executable(normalization_image_generator_node src/nodes/normalization_image_generator_node.cpp)
target_link_libraries(normalization_image_generator_node ${PROJECT_NAME})

add_executable(normalize_response src/nodes/normalize_response.cpp)
target_link_libraries(normalize_response ${catkin_LIBRARIES})

add_executable(rotate_image src/nodes/rotate_image.cpp)
target_link_libraries(rotate_image ${catkin_LIBRARIES})

add_executable(scale_image src/nodes/scale_image.cpp)
target_link_libraries(scale_image ${catkin_LIBRARIES})

add_executable(replace_colors src/nodes/replace_colors.cpp)
target_link_libraries(replace_colors ${catkin_LIBRARIES})

swri_nodelet_add_node(draw_polygon ${PROJECT_NAME} DrawPolygonNodelet)
target_link_libraries(draw_polygon ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

swri_nodelet_add_node(image_pub ${PROJECT_NAME} ImagePubNodelet)
target_link_libraries(image_pub ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

swri_nodelet_add_node(warp_image ${PROJECT_NAME} WarpImageNodelet)
target_link_libraries(warp_image ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

swri_nodelet_add_node(crosshairs ${PROJECT_NAME} CrosshairsNodelet)
target_link_libraries(crosshairs ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_geometry_util test/geometry_util.test test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelets
    blend_images
    contrast_stretch
    crosshairs
    draw_polygon
    draw_text
    dummy_image_publisher
    image_pub
    normalization_image_generator_node
    normalize_response
    replace_colors
    rotate_image
    scale_image
    warp_image
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
