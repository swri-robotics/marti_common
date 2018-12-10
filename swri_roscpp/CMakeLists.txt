cmake_minimum_required(VERSION 2.8.3)

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

### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
include_directories(include
  ${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS ${RUNTIME_DEPS}
  INCLUDE_DIRS include
  CFG_EXTRAS swri_roscpp-extras.cmake)

### Build Test Node ###
add_executable(subscriber_test src/nodes/subscriber_test.cpp)
target_link_libraries(subscriber_test ${catkin_LIBRARIES})

add_executable(latched_subscriber_test src/nodes/latched_subscriber_test.cpp)
target_link_libraries(latched_subscriber_test ${catkin_LIBRARIES})

add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
target_link_libraries(storing_subscriber_test ${catkin_LIBRARIES})

add_executable(service_server_test src/nodes/service_server_test.cpp)
target_link_libraries(service_server_test ${catkin_LIBRARIES})

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

