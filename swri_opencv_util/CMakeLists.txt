cmake_minimum_required(VERSION 3.10)
project(swri_opencv_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread random)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED core imgproc highgui)
find_package(swri_math_util)
  
add_library(${PROJECT_NAME} SHARED
  src/blend.cpp
  src/models.cpp
  src/model_fit.cpp
  src/show.cpp
  src/convert.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${swri_math_util_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC
  Boost::boost
  Boost::serialization
  Boost::thread
  cv_bridge::cv_bridge
  opencv_core
  opencv_imgproc
  opencv_highgui
  ${swri_math_util_LIBRARIES})

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
ament_export_dependencies(swri_math_util)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
