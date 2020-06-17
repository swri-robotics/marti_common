cmake_minimum_required(VERSION 3.0.2)
project(swri_nodelet)

set(BUILD_DEPS 
  nodelet
  roscpp
  std_msgs
)

set(RUNTIME_DEPS ${BUILD_DEPS})

find_package(Boost REQUIRED)
# Workaround for header-only Boost issue in older versions of cmake:
# https://gitlab.kitware.com/cmake/cmake/-/issues/18643
add_library(${PROJECT_NAME}_boost INTERFACE IMPORTED)
set_property(TARGET ${PROJECT_NAME}_boost PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
catkin_package(
  CATKIN_DEPENDS ${RUNTIME_DEPS}
  INCLUDE_DIRS include
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
  DEPENDS Boost
)

# nodelet.cpp.in must be copied before include(...) or clean builds will fail
file(COPY nodelet.cpp.in
   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
)
set(${PROJECT_NAME}_PREFIX ${CATKIN_DEVEL_PREFIX})  # Allows use of nodelet.cpp.in in swri_nodelet_add_node macro for this build
include(${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}-extras.cmake)  # Allows use of swri_nodelet_add_node macro for this build

include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_test src/test_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_boost
)

swri_nodelet_add_node(test_node ${PROJECT_NAME} TestNodelet)
target_link_libraries(test_node
  ${PROJECT_NAME}_test
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_manager.test DEPENDENCIES ${PROJECT_NAME}_test)
    add_rostest(test/test_standalone.test DEPENDENCIES ${PROJECT_NAME}_test)
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
