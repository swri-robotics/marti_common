cmake_minimum_required(VERSION 2.8.3)

project(swri_roscpp)

set(BUILD_DEPS
  diagnostic_updater 
  nav_msgs
  roscpp
  std_msgs
  std_srvs
)

set(RUNTIME_DEPS
  diagnostic_updater 
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
  INCLUDE_DIRS include)

### Build Test Node ###
add_executable(subscriber_test src/nodes/subscriber_test.cpp)
target_link_libraries(subscriber_test ${catkin_LIBRARIES})

add_executable(latched_subscriber_test src/nodes/latched_subscriber_test.cpp)
target_link_libraries(latched_subscriber_test ${catkin_LIBRARIES})

add_executable(service_server_test src/nodes/service_server_test.cpp)
target_link_libraries(service_server_test ${catkin_LIBRARIES})

add_executable(timer_test src/nodes/timer_test.cpp)
target_link_libraries(timer_test ${catkin_LIBRARIES})

### Install Test Node and Headers ###
install(TARGETS subscriber_test latched_subscriber_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

