cmake_minimum_required(VERSION 2.8.3)
project(swri_yaml_util)

find_package(catkin REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
message(STATUS "YAML-CPP VERSION: " ${YamlCpp_VERSION})
message(STATUS "YAML-CPP LIBRARY: " ${YamlCpp_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${YamlCpp_LIBRARIES}
)

if( ${YamlCpp_VERSION} MATCHES "0.[23].*")
  set(YAMLCPP_OLD_API ON)
  message(STATUS "  Using old yaml-cpp API.")
else()
  set(YAMLCPP_OLD_API OFF)
  message(STATUS "  Using new yaml-cpp API.")
endif()

configure_file(version.h.in ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/version.h)

find_package(Boost REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${YamlCpp_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/yaml_util.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

