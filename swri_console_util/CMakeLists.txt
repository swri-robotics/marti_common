cmake_minimum_required(VERSION 3.10)
project(swri_console_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/progress_bar.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake
  rclcpp
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
