cmake_minimum_required(VERSION 3.10)

# Geographiclib installs GeographicLibConfig.cmake to this non-standard location
# Doing this here, so that downstream users don't have to manually do find it when building against swri_transform_util
find_package(GeographicLib QUIET)

if (NOT ${GeographicLib_FOUND})
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
  find_package(GeographicLib REQUIRED)
endif()