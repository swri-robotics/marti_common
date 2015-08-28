cmake_minimum_required(VERSION 2.8.3)
project(swri_system_util)

find_package(catkin REQUIRED COMPONENTS
  roscpp)

find_package(Boost REQUIRED COMPONENTS filesystem regex)

if(${Boost_MAJOR_VERSION} LESS 2 AND ${Boost_MINOR_VERSION} LESS 48)
  add_definitions( -DBOOST_FILESYSTEM_VERSION=2 )
  message(STATUS "  Using Boost Filesystem Version 2")
else(${Boost_MAJOR_VERSION} LESS 2 AND ${Boost_MINOR_VERSION} LESS 48)
  add_definitions( -DBOOST_FILESYSTEM_VERSION=3 )
  message(STATUS "  Using Boost Filesystem Version 3")
endif(${Boost_MAJOR_VERSION} LESS 2 AND ${Boost_MINOR_VERSION} LESS 48)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
  
add_library(${PROJECT_NAME}
  src/file_util.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
    
  add_rostest_gtest(test_file_util launch/file_util.test test/test_file_util.cpp)
  target_link_libraries(test_file_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
