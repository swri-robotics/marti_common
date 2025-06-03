cmake_minimum_required(VERSION 3.10)

set(swri_roscpp_BIN "${swri_roscpp_DIR}/../../../bin/")

macro(generate_topic_service_files generated_files_list)
  set(options NOINSTALL)
  set(oneValueArgs DIRECTORY)
  set(multiValueArgs FILES)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "generate_topic_service_files() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY "topic_srv")
  endif()

  set(TMP_TOPIC_SRV_DIR ${ARG_DIRECTORY})

  find_package(marti_common_msgs REQUIRED)

  foreach (_file ${ARG_FILES})
    set(current_ReqOut ${_file}Request.msg)
    string(REPLACE ".srv" "" current_ReqOut ${current_ReqOut})
    set(current_ResOut ${_file}Response.msg)
    string(REPLACE ".srv" "" current_ResOut ${current_ResOut})

    set(TMP_MSG_DIR ${CMAKE_CURRENT_BINARY_DIR}/msg)
    set(TMP_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/srv)

    file(MAKE_DIRECTORY ${TMP_MSG_DIR})
    file(MAKE_DIRECTORY ${TMP_INCLUDE_DIR})

    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${TMP_TOPIC_SRV_DIR}/${_file})

    set(headerName ${_file})
    string(REPLACE ".srv" ".h" headerName ${headerName})
    execute_process(
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${swri_roscpp_BIN}service_splitter.py ${_file} ${TMP_MSG_DIR}/${current_ReqOut} ${TMP_MSG_DIR}/${current_ResOut} ${PROJECT_NAME} ${_file} ${TMP_INCLUDE_DIR}/${headerName}
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${TMP_TOPIC_SRV_DIR}"
    )

    set(TMP_MSG_FILES ${TMP_MSG_FILES} ${TMP_MSG_DIR}:${current_ResOut} ${TMP_MSG_DIR}:${current_ReqOut})

    # install the header
    install(FILES ${TMP_INCLUDE_DIR}/${headerName}
      DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/srv
    )

  endforeach()

  set(${generated_files_list} ${TMP_MSG_FILES})
endmacro()
