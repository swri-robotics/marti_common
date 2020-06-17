cmake_minimum_required(VERSION 3.0.2)
project(swri_route_util)

set(BUILD_DEPS 
  marti_common_msgs
  marti_nav_msgs
  roscpp
  swri_geometry_util
  swri_math_util
  swri_transform_util
  visualization_msgs
  )

set(RUNTIME_DEPS ${BUILD_DEPS})

find_package(Boost REQUIRED)
# Workaround for header-only Boost issue in older versions of cmake:
# https://gitlab.kitware.com/cmake/cmake/-/issues/18643
add_library(${PROJECT_NAME}_boost INTERFACE IMPORTED)
set_property(TARGET ${PROJECT_NAME}_boost PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})

### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
catkin_package(CATKIN_DEPENDS ${RUNTIME_DEPS}
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})

### Build Library ###
add_library(${PROJECT_NAME} 
  src/route.cpp
  src/route_point.cpp
  src/route_speeds.cpp
  src/util.cpp
  src/visualization.cpp
  )
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++0x")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_boost
)

### Install Libraries and Executables ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### Install Header Files ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

