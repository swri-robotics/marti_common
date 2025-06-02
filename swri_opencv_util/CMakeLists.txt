cmake_minimum_required(VERSION 3.10)
project(swri_opencv_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
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
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  opencv_core
  ${swri_math_util_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake OpenCV swri_math_util)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
