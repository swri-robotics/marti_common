cmake_minimum_required(VERSION 3.0.2)
project(rosman)

set(RUN_DEPS
  marti_common_msgs
  rosgraph
  rospy
  rostopic
)

find_package(catkin REQUIRED
)

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPS}
)

catkin_python_setup()

