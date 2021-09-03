cmake_minimum_required(VERSION 3.0.2)
project(rosman)

set(RUN_DEPS
  marti_introspection_msgs
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

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_env_hooks(15.rosman SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

