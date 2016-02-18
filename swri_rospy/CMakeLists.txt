cmake_minimum_required(VERSION 2.8.3)

project(swri_rospy)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/single_threaded_example
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
