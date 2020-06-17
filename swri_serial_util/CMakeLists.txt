cmake_minimum_required(VERSION 3.0.2)
project(swri_serial_util)

find_package(Boost REQUIRED)
# Workaround for header-only Boost issue in older versions of cmake:
# https://gitlab.kitware.com/cmake/cmake/-/issues/18643
add_library(${PROJECT_NAME}_boost INTERFACE IMPORTED)
set_property(TARGET ${PROJECT_NAME}_boost PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME} 
    src/serial_port.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_boost
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

