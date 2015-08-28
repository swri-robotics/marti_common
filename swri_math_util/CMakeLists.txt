cmake_minimum_required(VERSION 2.8.3)
project(swri_math_util)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost COMPONENTS random) 
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions( -DBOOST_1_46 )
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME}
  src/math_util.cpp
  src/trig_util.cpp
  src/random.cpp
  src/interpolation_1d.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest_gtest(test_trig_util launch/trig_util.test test/test_trig_util.cpp)
  target_link_libraries(test_trig_util ${PROJECT_NAME})

  add_rostest_gtest(test_math_util launch/math_util.test test/test_math_util.cpp)
  target_link_libraries(test_math_util ${PROJECT_NAME})

  add_rostest_gtest(test_random launch/random.test test/test_random.cpp)
  target_link_libraries(test_random ${PROJECT_NAME})
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

