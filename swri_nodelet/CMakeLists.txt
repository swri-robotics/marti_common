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
  INCLUDE_DIRS include
  CFG_EXTRAS swri_nodelet-extras.cmake
)

# nodelet.cpp.in must be copied before include(...) or clean builds will fail
file(COPY nodelet.cpp.in
   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
)
set(swri_nodelet_PREFIX ${CATKIN_DEVEL_PREFIX})  # Allows use of nodelet.cpp.in in swri_nodelet_add_node macro for this build
include(${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/swri_nodelet-extras.cmake)  # Allows use of swri_nodelet_add_node macro for this build

include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_library(swri_nodelet_test src/test_nodelet.cpp)
target_link_libraries(swri_nodelet_test ${catkin_LIBRARIES})

swri_nodelet_add_node(test_node swri_nodelet TestNodelet)
target_link_libraries(test_node swri_nodelet_test)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_manager.test DEPENDENCIES swri_nodelet_test)
    add_rostest(test/test_standalone.test DEPENDENCIES swri_nodelet_test)
    add_rostest(test/test_node_wrapper.test DEPENDENCIES test_node)
endif()

install(PROGRAMS 
  nodes/nodelet
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet.cpp.in
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
