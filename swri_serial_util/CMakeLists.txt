cmake_minimum_required(VERSION 3.10)
project(swri_serial_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)

ament_package()
