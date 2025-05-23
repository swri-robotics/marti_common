cmake_minimum_required(VERSION 3.10)
project(swri_opencv_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED core imgproc highgui)
find_package(swri_math_util REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/blend.cpp
  src/models.cpp
  src/model_fit.cpp
  src/show.cpp
  src/convert.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  opencv_core
  opencv_highgui
  opencv_imgproc
  ${swri_math_util_TARGETS})

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(OpenCV)
ament_export_dependencies(swri_math_util)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
