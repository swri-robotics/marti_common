cmake_minimum_required(VERSION 3.10)
project(swri_image_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

find_package(OpenCV REQUIRED COMPONENTS core stitching)

find_package(Boost REQUIRED COMPONENTS filesystem system random)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
  src/draw_util.cpp
  src/geometry_util.cpp
  src/image_matching.cpp
  src/image_normalization.cpp
  src/image_warp_util.cpp
  src/replace_colors.cpp
  src/rolling_normalization.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_property(TARGET ${PROJECT_NAME}
  PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}
  Boost::boost
  Boost::filesystem
  Boost::random
  Boost::system
  Eigen3::Eigen
  opencv_core
  opencv_stitching
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
  rclcpp_components
  std_msgs
  swri_geometry_util
  swri_math_util
  swri_opencv_util
  swri_roscpp
  tf2
)

add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/blend_images_node.cpp
  src/nodes/crosshairs_node.cpp
  src/nodes/contrast_stretch_node.cpp
  src/nodes/draw_polygon_node.cpp
  src/nodes/draw_text_node.cpp
  src/nodes/dummy_image_publisher_node.cpp
  src/nodes/image_pub_node.cpp
  src/nodes/normalization_image_node.cpp
  src/nodes/normalize_response_node.cpp
  #src/nodes/replace_colors_node.cpp
  src/nodes/rotate_image_node.cpp
  src/nodes/scale_image_node.cpp
  src/nodes/warp_image_node.cpp
)
target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::BlendImagesNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ContrastStretchNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::CrosshairsNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::DrawPolygonNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::DrawTextNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::DummyImagePublisherNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ImagePubNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::NormalizationImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::NormalizeResponseNode")
# rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ReplaceColorsNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::RotateImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ScaleImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::WarpImageNode")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_nodes
  ament_index_cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry_util test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
    ${PROJECT_NAME}_nodes
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_nodes)

ament_package()
