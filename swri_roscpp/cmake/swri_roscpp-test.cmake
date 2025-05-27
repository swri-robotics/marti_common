if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ## Storing subscriber test case
  add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
  target_include_directories(storing_subscriber_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_link_libraries(storing_subscriber_test
    ${diagnostic_updater_TARGETS}
    ${marti_common_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    )

  ## Service server test
  add_executable(service_server_test src/nodes/service_server_test.cpp)
  target_include_directories(service_server_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(service_server_test
    ${diagnostic_updater_TARGETS}
    ${marti_common_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    )

  ## Timer test
  add_executable(timer_test src/nodes/timer_test.cpp)
  target_include_directories(timer_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(timer_test
    ${diagnostic_updater_TARGETS}
    ${marti_common_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    )

  rosidl_generate_interfaces(${PROJECT_NAME}_test
    msg/TestTopicServiceRequest.msg
    msg/TestTopicServiceResponse.msg
    DEPENDENCIES std_msgs marti_common_msgs
    LIBRARY_NAME ${PROJECT_NAME}
  )
  ament_add_gtest(topic_service_test_server test/topic_service_test.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_test" "rosidl_typesupport_cpp")
  target_link_libraries(topic_service_test_server
    "${cpp_typesupport_target}"
    ${marti_common_msgs_TARGETS}
    rclcpp::rclcpp)

  target_include_directories(topic_service_test_server
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

  ament_export_dependencies(rosidl_default_runtime)
endif()