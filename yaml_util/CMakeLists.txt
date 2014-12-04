cmake_minimum_required(VERSION 2.4.6)
project(yaml_util)

find_package(catkin REQUIRED COMPONENTS build_tools)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} yaml-cpp
)

execute_process(
  COMMAND rospack find build_tools
  ERROR_VARIABLE __null
  OUTPUT_VARIABLE build_tools_PACKAGE_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)




find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
message ("YAML-CPP VERSION: " ${YamlCpp_VERSION})
if( ${YamlCpp_VERSION} MATCHES "0.[23].*")
  file(WRITE ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/version.h "#define YAMLCPP_OLD_API")
  message ("  Using old yaml-cpp API.")
else( ${YamlCpp_VERSION} MATCHES "0.[23].*")
  file(WRITE ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/version.h "#undef YAMLCPP_OLD_API")
  message ("  Using new yaml-cpp API.")
endif( ${YamlCpp_VERSION} MATCHES "0.[23].*")

include_directories(include ${catkin_INCLUDE_DIRS} ${YamlCpp_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/yaml_util.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YamlCpp_LIBRARY})
  
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

