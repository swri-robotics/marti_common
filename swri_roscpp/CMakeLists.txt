cmake_minimum_required(VERSION 3.10)

project(swri_roscpp)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(diagnostic_updater REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# TODO pjr What to do about swri_roscpp-extras.cmake?

### Build Test Node ###
add_executable(subscriber_test src/nodes/subscriber_test.cpp)
ament_target_dependencies(subscriber_test
  "diagnostic_updater"
  "marti_common_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)

add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
ament_target_dependencies(storing_subscriber_test
  "diagnostic_updater"
  "marti_common_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "std_srvs"
  )

add_executable(service_server_test src/nodes/service_server_test.cpp)
ament_target_dependencies(service_server_test
  "diagnostic_updater"
  "marti_common_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "std_srvs"
  )

add_executable(timer_test src/nodes/timer_test.cpp)
ament_target_dependencies(timer_test
  "diagnostic_updater"
  "marti_common_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "std_srvs"
  )

add_executable(param_example src/nodes/param_example.cpp)
ament_target_dependencies(param_example
  "diagnostic_updater"
  "marti_common_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "std_srvs"
  )

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
#  find_package(rosidl_default_generators REQUIRED)
#  rosidl_generate_interfaces(${PROJECT_NAME}
#    msg/TestTopicServiceRequest.msg
#    msg/TestTopicServiceResponse.msg
#    DEPENDENCIES std_msgs
#    )
#  add_rostest_gtest(test_params
#                    test/params.test
#                    test/params.cpp)
#  target_link_libraries(test_params ${catkin_LIBRARIES})
#
#  ### Test the TopicService server and client ###
#  # This is a little different from the way tests are normally declared because we
#  # have a single binary that contains both server and client tests, but the tests
#  # themselves are run in separate .test files.
#  add_executable(test_topic_service
#    test/topic_service_test.cpp)
#  target_link_libraries(test_topic_service
#    ${catkin_LIBRARIES}
#    ${GTEST_LIBRARIES})
#  add_dependencies(test_topic_service ${${PROJECT_NAME}_EXPORTED_TARGETS})
#  add_rostest(test/topic_service_client_test.test DEPENDENCIES test_topic_service)
#  add_rostest(test/topic_service_server_test.test DEPENDENCIES test_topic_service)
endif()

### Install Test Node and Headers ###
install(TARGETS subscriber_test param_example
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
ament_export_include_directories(include
  ${diagnostic_updater_INCLUDE_DIRS}
  ${marti_common_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
)
ament_export_libraries(${PROJECT_NAME}
  ${diagnostic_updater_LIBRARIES}
  ${marti_common_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
)

ament_package()
