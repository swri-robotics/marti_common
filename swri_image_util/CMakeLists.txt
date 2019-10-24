cmake_minimum_required(VERSION 3.10)
project(swri_image_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swri_geometry_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_opencv_util REQUIRED)
find_package(swri_roscpp REQUIRED)
find_package(tf2 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem system random)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/draw_util.cpp
  src/geometry_util.cpp
  src/image_matching.cpp
  src/image_normalization.cpp
  src/image_warp_util.cpp
  src/replace_colors.cpp
  src/rolling_normalization.cpp
)
set_property(TARGET ${PROJECT_NAME}
  PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}
  Boost::boost
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  camera_calibration_parsers
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  message_filters
  nav_msgs
  rclcpp
  std_msgs
  swri_geometry_util
  swri_math_util
  swri_opencv_util
  swri_roscpp
  tf2
)

add_library(${PROJECT_NAME}_nodelets SHARED
  src/nodes/blend_images_nodelet.cpp
  src/nodes/crosshairs_nodelet.cpp
  src/nodes/contrast_stretch_nodelet.cpp
  src/nodes/draw_polygon_nodelet.cpp
  src/nodes/draw_text_nodelet.cpp
  src/nodes/image_pub_nodelet.cpp
  src/nodes/normalize_response_nodelet.cpp
#  src/nodes/replace_colors_nodelet.cpp
  src/nodes/rotate_image_nodelet.cpp
  src/nodes/scale_image_nodelet.cpp
  src/nodes/warp_image_nodelet.cpp
)
target_compile_definitions(${PROJECT_NAME}_nodelets
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::BlendImagesNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::ContrastStretchNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::CrosshairsNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::DrawPolygonNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::DrawTextNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::ImagePubNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::NormalizeResponseNodelet")
# rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::ReplaceColorsNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::RotateImageNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::ScaleImageNodelet")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelets "swri_image_util::WarpImageNodelet")
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME})

add_executable(dummy_image_publisher src/nodes/dummy_image_publisher.cpp)
target_link_libraries(dummy_image_publisher ${catkin_LIBRARIES})

add_executable(normalization_image_generator_node src/nodes/normalization_image_generator_node.cpp)
target_link_libraries(normalization_image_generator_node ${PROJECT_NAME})

#swri_nodelet_add_node(draw_polygon ${PROJECT_NAME} DrawPolygonNodelet)
#target_link_libraries(draw_polygon ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#
#swri_nodelet_add_node(image_pub ${PROJECT_NAME} ImagePubNodelet)
#target_link_libraries(image_pub ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#
#swri_nodelet_add_node(warp_image ${PROJECT_NAME} WarpImageNodelet)
#target_link_libraries(warp_image ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#
#swri_nodelet_add_node(crosshairs ${PROJECT_NAME} CrosshairsNodelet)
#target_link_libraries(crosshairs ${PROJECT_NAME}_nodelets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry_util test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelets
    # blend_images
    # contrast_stretch
    # crosshairs
    # draw_polygon
    # draw_text
    # dummy_image_publisher
    # image_pub
    # normalization_image_generator_node
    # normalize_response
    # replace_colors
    # rotate_image
    # scale_image
    # warp_image
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_nodelets)

ament_package()