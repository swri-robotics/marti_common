cmake_minimum_required(VERSION 3.16)
project(marti_data_structures)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
