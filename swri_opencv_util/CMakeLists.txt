cmake_minimum_required(VERSION 3.14.4)
project(swri_opencv_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED core imgproc highgui)
find_package(swri_math_util REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/blend.cpp
  src/models.cpp
  src/model_fit.cpp
  src/convert.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  opencv_core
  ${swri_math_util_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE
  opencv_imgproc
)

# ShowScaled() opens HighGUI windows, so it lives in its own library.  HighGUI
# loads its GUI toolkit (Qt5 on Ubuntu) as soon as it is linked, and a Qt6
# application that merely links swri_opencv_util would otherwise end up with
# both Qt versions in one process.
add_library(${PROJECT_NAME}_show SHARED
  src/show.cpp)

target_include_directories(${PROJECT_NAME}_show PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME}_show PUBLIC
  opencv_core)
target_link_libraries(${PROJECT_NAME}_show PRIVATE
  opencv_highgui
  opencv_imgproc
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS ${PROJECT_NAME}_show
  EXPORT export_${PROJECT_NAME}_show
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Kept out of ament_export_targets() so it is not added to
# ${PROJECT_NAME}_TARGETS; downstream packages opt in by linking
# swri_opencv_util::swri_opencv_util_show.  The config extras load it.
install(EXPORT export_${PROJECT_NAME}_show
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake OpenCV swri_math_util)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # TODO(#818): Enable the remaining linters once the code complies with them.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)
