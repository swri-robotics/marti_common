cmake_minimum_required(VERSION 2.8.3)
project(swri_opencv_util)

find_package(catkin REQUIRED COMPONENTS cv_bridge swri_math_util)

find_package(OpenCV REQUIRED core imgproc highgui)

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost REQUIRED COMPONENTS serialization thread) 
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions( -DBOOST_1_46 )
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge swri_math_util
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME}
  src/blend.cpp
  src/models.cpp
  src/model_fit.cpp
  src/show.cpp
  src/convert.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
