cmake_minimum_required(VERSION 3.10)

project(swri_roscpp)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# TODO pjr What to do about swri_roscpp-extras.cmake?
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets("export_${PROJECT_NAME}")

# ### Build Test Node ###
add_executable(subscriber_test src/nodes/subscriber_test.cpp)
target_include_directories(subscriber_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(subscriber_test
  diagnostic_updater
  marti_common_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
)

add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
target_include_directories(storing_subscriber_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(storing_subscriber_test
  diagnostic_updater
  marti_common_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  )

add_executable(service_server_test src/nodes/service_server_test.cpp)
target_include_directories(service_server_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(service_server_test
  diagnostic_updater
  marti_common_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  )

add_executable(timer_test src/nodes/timer_test.cpp)
target_include_directories(timer_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(timer_test
  diagnostic_updater
  marti_common_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  )

  # Disabling Foxy and Galactic tests until https://github.com/ros2/rosidl/pull/639 is merged
if (BUILD_TESTING AND ("$ENV{ROS_DISTRO}" STRGREATER "galactic"))
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system)

  rosidl_generate_interfaces(${PROJECT_NAME}_test
    msg/TestTopicServiceRequest.msg
    msg/TestTopicServiceResponse.msg
    DEPENDENCIES std_msgs marti_common_msgs
    LIBRARY_NAME ${PROJECT_NAME}
  )

  ament_add_gtest(topic_service_test_server test/topic_service_test.cpp)

  # Humble and onwards
  if ("$ENV{ROS_DISTRO}" STRGREATER "galactic")
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_test" "rosidl_typesupport_cpp")
    target_link_libraries(topic_service_test_server
      Boost::system
      "${cpp_typesupport_target}"
    )
  else()
    rosidl_target_interfaces(topic_service_test_server "${PROJECT_NAME}_test" "rosidl_typesupport_cpp")
    target_link_libraries(topic_service_test_server
      Boost::system
      swri_roscpp_test__rosidl_typesupport_cpp
    )
  endif()

  ament_target_dependencies(topic_service_test_server
    rclcpp
    marti_common_msgs
  )
  target_include_directories(topic_service_test_server
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_export_dependencies(rosidl_default_runtime)
endif()

### Install Test Node and Headers ###
install(TARGETS subscriber_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS scripts/service_splitter.py
  DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION launch
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diagnostic_updater)
ament_export_dependencies(marti_common_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_include_directories(include)

ament_package(CONFIG_EXTRAS cmake/swri_roscpp-extras.cmake )
