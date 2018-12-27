cmake_minimum_required(VERSION 2.8.3)

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

  find_package(catkin REQUIRED COMPONENTS marti_common_msgs)

  foreach (_file ${ARG_FILES})
    set(current_ReqOut ${_file}Request.msg)
    string(REPLACE ".srv" "" current_ReqOut ${current_ReqOut})
    set(current_ResOut ${_file}Response.msg)
    string(REPLACE ".srv" "" current_ResOut ${current_ResOut})

    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg)

    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/topic_srv/${_file} _fileContents)
    
    string(REPLACE "---" ";" CONTENTS_LIST ${_fileContents})
   
    #include(${CMAKE_CURRENT_SOURCE_DIR}/topic_srv/${_file})

    list(GET CONTENTS_LIST 0 MSG_REQUEST)
    list(GET CONTENTS_LIST 1 MSG_RESPONSE)

    file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg/${current_ReqOut} "marti_common_msgs/ServiceHeader srv_header\n" ${MSG_REQUEST})
    file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg/${current_ResOut} "marti_common_msgs/ServiceHeader srv_header\n" ${MSG_RESPONSE})

    set(headerName ${_file})
    string(REPLACE ".srv" ".h" headerName ${headerName})
    add_custom_target(${_file}thing
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${swri_roscpp_BIN}service_splitter.py ${_file} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg/${current_ReqOut} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg/${current_ResOut} ${PROJECT_NAME} ${_file} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${headerName}
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/topic_srv"
    )
    add_dependencies(${catkin_EXPORTED_TARGETS}  
         ${_file}thing)

    add_message_files(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/msg FILES
      ${current_ReqOut} 
      ${current_ResOut}
    )

    foreach (_proj ${catkin_EXPORTED_TARGETS})
      add_dependencies(${_proj} ${_file}thing)
    endforeach()

    # install the header
    install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${headerName}
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
    
  endforeach()
endmacro()
