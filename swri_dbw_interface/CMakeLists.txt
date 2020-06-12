cmake_minimum_required(VERSION 3.0.2)
project(swri_dbw_interface)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
