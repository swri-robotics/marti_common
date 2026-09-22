cmake_minimum_required(VERSION 3.14.4)
project(swri_image_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED core highgui imgcodecs imgproc stitching)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swri_geometry_util REQUIRED)
find_package(swri_opencv_util REQUIRED)
find_package(tf2 REQUIRED)

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
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${swri_opencv_util_TARGETS}
  opencv_core
  opencv_imgproc
  opencv_stitching
  rclcpp::rclcpp
  tf2::tf2)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen ${swri_geometry_util_TARGETS})

# The functions that show images in HighGUI windows live in their own library.
# HighGUI loads its GUI toolkit (Qt5 on Ubuntu) as soon as it is linked, and a
# Qt6 application that merely links swri_image_util would otherwise end up with
# both Qt versions in one process.
add_library(${PROJECT_NAME}_display SHARED
  src/draw_util_display.cpp
  src/image_warp_util_display.cpp
)
target_link_libraries(${PROJECT_NAME}_display PUBLIC ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_display PRIVATE
  opencv_highgui
  opencv_imgproc
  swri_opencv_util::swri_opencv_util_show)

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
  src/nodes/replace_colors_node.cpp
  src/nodes/rotate_image_node.cpp
  src/nodes/scale_image_node.cpp
  src/nodes/warp_image_node.cpp)

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
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ReplaceColorsNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::RotateImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::ScaleImageNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "swri_image_util::WarpImageNode")

target_link_libraries(${PROJECT_NAME}_nodes PRIVATE
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp
    cv_bridge::cv_bridge
    image_transport::image_transport
    opencv_imgcodecs
    message_filters::message_filters
    rclcpp_components::component
    rclcpp_components::component_manager
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
)

# Iron and later switched some cv_bridge files to .hpp from .h
if("${cv_bridge_VERSION}" VERSION_LESS "3.3.0")
  target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE "-DUSE_CVBRIDGE_H_FILES")
endif()

# The image_transport::RequiredInterfaces (NodeInterfaces) + rclcpp::QoS API was
# introduced in image_transport 6.4.0. Older versions only provide the legacy
# (rclcpp::Node *, rmw_qos_profile_t) API, so compile the legacy code path for
# them. Newer versions fall through to the default path and use the current,
# non-deprecated API.
if("${image_transport_VERSION}" VERSION_LESS "6.4.0")
  target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE "-DUSE_LEGACY_IMAGE_TRANSPORT_API")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # TODO(#818): Enable the remaining linters once the code complies with them.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  # Parse .h headers as C++ rather than C.
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS LANGUAGE CPP)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry_util test/test_geometry_util.cpp)
  target_link_libraries(test_geometry_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS ${PROJECT_NAME}_display
  EXPORT export_${PROJECT_NAME}_display
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Kept out of ament_export_targets() so it is not added to
# ${PROJECT_NAME}_TARGETS; downstream packages opt in by linking
# swri_image_util::swri_image_util_display.  The config extras load it.
install(EXPORT export_${PROJECT_NAME}_display
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

install(TARGETS ${PROJECT_NAME}_nodes
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV
  rclcpp
  swri_opencv_util
  tf2
)
ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)
