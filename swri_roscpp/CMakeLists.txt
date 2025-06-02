cmake_minimum_required(VERSION 3.10)

project(swri_roscpp)

# Default to C++ version appropriate for the ROS distribution
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17) 
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}_library INTERFACE)

target_link_libraries(${PROJECT_NAME}_library INTERFACE
  ${diagnostic_updater_TARGETS}
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
)
target_include_directories(${PROJECT_NAME}_library INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}_library
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

include(cmake/swri_roscpp-test.cmake)

add_executable(subscriber_test src/nodes/subscriber_test.cpp)
target_link_libraries(subscriber_test
  ${PROJECT_NAME}_library
  ${nav_msgs_TARGETS}
)
### Install Test Node and Headers ###
install(TARGETS subscriber_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS scripts/service_splitter.py
  DESTINATION bin
)
install(DIRECTORY launch/
  DESTINATION launch
)

ament_export_dependencies(ament_cmake
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  std_msgs
)

ament_package(CONFIG_EXTRAS cmake/swri_roscpp-extras.cmake )
