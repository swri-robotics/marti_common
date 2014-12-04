cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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

rosbuild_add_library(${PROJECT_NAME}
  src/yaml_util.cpp
)

target_link_libraries(${PROJECT_NAME} ${YAMLCPP_LIBRARY})
