if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system)
  ## Storing subscriber test case
  add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
  target_include_directories(storing_subscriber_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${diagnostic_updater_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
  )
  target_link_libraries(storing_subscriber_test
    ${diagnostic_updater_LIBRARIES}
    ${marti_common_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
    )

  ## Service server test
  add_executable(service_server_test src/nodes/service_server_test.cpp)
  target_include_directories(service_server_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${diagnostic_updater_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
  )
  target_link_libraries(service_server_test
    ${diagnostic_updater_LIBRARIES}
    ${marti_common_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
    )

  ## Timer test
  add_executable(timer_test src/nodes/timer_test.cpp)
  target_include_directories(timer_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${diagnostic_updater_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
  )
  target_link_libraries(timer_test
    ${diagnostic_updater_LIBRARIES}
    ${marti_common_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
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
    Boost::system
    "${cpp_typesupport_target}"
  )
  target_link_libraries(topic_service_test_server
    ${rclcpp_LIBRARIES}
    ${marti_common_msgs_LIBRARIES}
  )
  target_include_directories(topic_service_test_server
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${marti_common_msgs_INCLUDE_DIRS}
  )

  ament_export_dependencies(rosidl_default_runtime)
endif()