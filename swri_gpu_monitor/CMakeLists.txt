cmake_minimum_required(VERSION 2.8.3)
project(swri_gpu_monitor)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/gpu_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
