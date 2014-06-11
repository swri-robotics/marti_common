# Default build script for MARTI ROS packages
# Anything in this script will be executed for all packages that include it

rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/log_version.cmake)
include(${build_tools_PACKAGE_PATH}/scripts/version_file.cmake)
