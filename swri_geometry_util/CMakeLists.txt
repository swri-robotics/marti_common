cmake_minimum_required(VERSION 2.8.3)

project(swri_geometry_util)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  roscpp
  tf
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
find_package(GEOS REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
find_package(OpenCV 2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf
  DEPENDS
    OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${GEOS_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/cubic_spline.cpp
  src/geometry_util.cpp
  src/intersection.cpp
  src/polygon.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GEOS_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
