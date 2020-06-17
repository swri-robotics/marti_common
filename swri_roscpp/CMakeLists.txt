cmake_minimum_required(VERSION 3.0.2)

project(swri_roscpp)

set(BUILD_DEPS
  diagnostic_updater
  dynamic_reconfigure 
  marti_common_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
)

set(RUNTIME_DEPS
  diagnostic_updater 
  dynamic_reconfigure
  marti_common_msgs
  nav_msgs 
  roscpp
  std_msgs
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIR}
)

if (CATKIN_ENABLE_TESTING)
  find_package(message_generation REQUIRED)
  find_package(message_runtime REQUIRED)
  add_message_files(DIRECTORY msg FILES
    TestTopicServiceRequest.msg
    TestTopicServiceResponse.msg
    )
  generate_messages(DEPENDENCIES marti_common_msgs)
endif()

catkin_package(CATKIN_DEPENDS ${RUNTIME_DEPS}
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES ${YamlCpp_LIBRARIES}
  CFG_EXTRAS swri_roscpp-extras.cmake)

### Build Test Node ###
add_executable(dynamic_parameters_test_node src/nodes/dynamic_parameters_test_node.cpp)
target_link_libraries(dynamic_parameters_test_node
  ${catkin_LIBRARIES}
  ${YamlCpp_LIBRARIES}
  Boost::thread
)
set_target_properties(dynamic_parameters_test_node PROPERTIES COMPILE_FLAGS "-std=c++11")

add_executable(subscriber_test src/nodes/subscriber_test.cpp)
target_link_libraries(subscriber_test ${catkin_LIBRARIES})

add_executable(latched_subscriber_test src/nodes/latched_subscriber_test.cpp)
target_link_libraries(latched_subscriber_test ${catkin_LIBRARIES})

add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
target_link_libraries(storing_subscriber_test ${catkin_LIBRARIES})

add_executable(service_server_test src/nodes/service_server_test.cpp)
target_link_libraries(service_server_test
  ${catkin_LIBRARIES}
  Boost::thread
)

add_executable(timer_test src/nodes/timer_test.cpp)
target_link_libraries(timer_test ${catkin_LIBRARIES})

add_executable(param_example src/nodes/param_example.cpp)
target_link_libraries(param_example ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_params
                    test/params.test
                    test/params.cpp)
  target_link_libraries(test_params ${catkin_LIBRARIES})

  add_rostest_gtest(test_dynamic_parameters
    test/test_dynamic_parameters.test
    test/test_dynamic_parameters.cpp)
  target_link_libraries(test_dynamic_parameters ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES})
  set_target_properties(test_dynamic_parameters PROPERTIES COMPILE_FLAGS "-std=c++11")

  ### Test the TopicService server and client ###
  # This is a little different from the way tests are normally declared because we
  # have a single binary that contains both server and client tests, but the tests
  # themselves are run in separate .test files.
  add_executable(test_topic_service
    test/topic_service_test.cpp)
  target_link_libraries(test_topic_service
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    Boost::thread
  )
  add_dependencies(test_topic_service ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_rostest(test/topic_service_client_test.test DEPENDENCIES test_topic_service)
  add_rostest(test/topic_service_server_test.test DEPENDENCIES test_topic_service)
endif()

### Install Test Node and Headers ###
install(TARGETS subscriber_test latched_subscriber_test param_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS  scripts/service_splitter.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

