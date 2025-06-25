cmake_minimum_required(VERSION 3.10)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
# Doing this here, so that downstream users don't have to manually do this to build against swri_transform_util
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
