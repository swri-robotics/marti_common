cmake_minimum_required(VERSION 3.0.2)
project(swri_opencv_util)

find_package(catkin REQUIRED COMPONENTS cv_bridge swri_math_util)

find_package(OpenCV REQUIRED core imgproc highgui)

find_package(Boost REQUIRED COMPONENTS serialization thread) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge swri_math_util
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME}
  src/blend.cpp
  src/models.cpp
  src/model_fit.cpp
  src/show.cpp
  src/convert.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Boost::serialization
  Boost::thread
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
