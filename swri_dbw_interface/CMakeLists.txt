cmake_minimum_required(VERSION 3.10)
project(swri_dbw_interface)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)

ament_package()