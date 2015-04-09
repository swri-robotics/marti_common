cmake_minimum_required(VERSION 2.4.6)
project(can_util)

find_package(catkin REQUIRED COMPONENTS 
  roscpp rospy marti_can_msgs build_tools)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS libqt4
  CATKIN_DEPENDS roscpp rospy marti_can_msgs
)

execute_process(
  COMMAND rospack find build_tools
  ERROR_VARIABLE __null
  OUTPUT_VARIABLE build_tools_PACKAGE_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(can_bus_example src/nodes/can_bus_example.cpp)
add_dependencies(can_bus_example marti_can_msgs_generate_messages_cpp)
target_link_libraries(can_bus_example ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS can_bus_example
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
