cmake_minimum_required(VERSION 3.0.2)

set(swri_roscpp_SHARE ${swri_roscpp_PREFIX}/@CATKIN_PACKAGE_SHARE_DESTINATION@)

@[if DEVELSPACE]@
# bin and template dir variables in develspace
set(swri_roscpp_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/")
@[else]@
# bin and template dir variables in installspace
set(swri_roscpp_BIN "${swri_roscpp_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/")
@[end if]@

macro(add_topic_service_files)
  cmake_parse_arguments(ARG "NOINSTALL" "DIRECTORY" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "add_service_files() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY "topic_srv")
  endif()

  set(TMP_TOPIC_SRV_DIR ${ARG_DIRECTORY})

  find_package(catkin REQUIRED COMPONENTS marti_common_msgs)

  foreach (_file ${ARG_FILES})
    set(current_ReqOut ${_file}Request.msg)
    string(REPLACE ".srv" "" current_ReqOut ${current_ReqOut})
    set(current_ResOut ${_file}Response.msg)
    string(REPLACE ".srv" "" current_ResOut ${current_ResOut})

    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg)
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/)

    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${TMP_TOPIC_SRV_DIR}/${_file})

    set(headerName ${_file})
    string(REPLACE ".srv" ".h" headerName ${headerName})
    safe_execute_process(
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${swri_roscpp_BIN}service_splitter.py ${_file} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg/${current_ReqOut} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg/${current_ResOut} ${PROJECT_NAME} ${_file} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${headerName}
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${TMP_TOPIC_SRV_DIR}"
    )

    add_message_files(BASE_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg FILES
      ${current_ReqOut} 
      ${current_ResOut}
    )

    # install the header
    install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${headerName}
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
    
  endforeach()
endmacro()
