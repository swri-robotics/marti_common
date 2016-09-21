cmake_minimum_required(VERSION 2.8.3)
project(swri_nodelet)

set(BUILD_DEPS 
  nodelet
  roscpp
  std_msgs
)

set(RUNTIME_DEPS ${BUILD_DEPS})

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
catkin_package(
  CATKIN_DEPENDS ${RUNTIME_DEPS}
)

add_library(swri_nodelet_test src/test_nodelet.cpp)
target_link_libraries(swri_nodelet_test ${catkin_LIBRARIES})

install(PROGRAMS 
  nodes/nodelet
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
