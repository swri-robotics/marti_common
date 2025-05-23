cmake_minimum_required(VERSION 3.10)
project(swri_system_util)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/file_util.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  include_directories(${ament_index_cpp_INCLUDE_DIRS})
  ament_add_gtest(test_file_util test/test_file_util.cpp)
  target_link_libraries(test_file_util
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp)
endif()

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(ament_cmake)

ament_package()
