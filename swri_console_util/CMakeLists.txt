cmake_minimum_required(VERSION 3.10)
project(swri_console_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} src/progress_bar.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

ament_package()