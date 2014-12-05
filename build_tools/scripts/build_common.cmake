# Default build script for MARTI ROS packages
# Anything in this script will be executed for all packages that include it

execute_process(
  COMMAND rospack find build_tools
  ERROR_VARIABLE __null
  OUTPUT_VARIABLE build_tools_PACKAGE_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
  
include(${build_tools_PACKAGE_PATH}/scripts/log_version.cmake)
include(${build_tools_PACKAGE_PATH}/scripts/version_file.cmake)
