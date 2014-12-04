cmake_minimum_required(VERSION 2.4.6)
project(build_tools)

find_package(catkin REQUIRED)
catkin_package()

include(scripts/build_common.cmake)
