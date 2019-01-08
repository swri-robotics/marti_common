cmake_minimum_required(VERSION 2.8.3)
project(swri_visualization_util)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  marti_nav_msgs
  visualization_msgs
  roscpp)


### Package Finding Tools ###
##find_package(PkgConfig REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS
      marti_nav_msgs
      visualization_msgs
      roscpp
)

# Build library
#add_library(${PROJECT_NAME} 
##  src/basler_camera.cpp
##  src/camera.cpp 
##  src/nodelets/adaptive_auto_exposure_nodelet.cpp
##  src/nodelets/basler_camera_nodelet.cpp
#)
##set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS -Wl,--as-needed)
##target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(obstacle_to_marker_node src/nodes/obstacle_to_marker_node.cpp)
target_link_libraries(obstacle_to_marker_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS #${PROJECT_NAME} 
        obstacle_to_marker_node
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
 # DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

