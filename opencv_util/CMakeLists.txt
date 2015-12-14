cmake_minimum_required(VERSION 2.8.3)
project(opencv_util)

find_package(catkin REQUIRED COMPONENTS math_util)

find_package(OpenCV REQUIRED)

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost COMPONENTS random) 
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions( -DBOOST_1_46 )
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS math_util
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME} 
  src/models.cpp
  src/model_fit.cpp
  src/show.cpp
  src/convert.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

#Python setup
catkin_python_setup()
set(Python_ADDITIONAL_VERSIONS 2.7)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# Build the _opencv_util python extension
link_libraries(${PYTHON_LIBRARIES})
add_library(py_opencv_util
  src/python_modules/py_opencv_util.cpp
  )
add_dependencies(py_opencv_util ${PROJECT_NAME}_gencpp)
set_target_properties(py_opencv_util PROPERTIES OUTPUT_NAME opencv_util PREFIX "_" SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/opencv_util)
set_target_properties(py_opencv_util PROPERTIES COMPILE_FLAGS "-fno-strict-aliasing")
target_link_libraries(py_opencv_util ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
