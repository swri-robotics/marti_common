cmake_minimum_required(VERSION 3.14.4)
project(swri_dbw_interface)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # TODO(#818): Enable the remaining linters once the code complies with them.
  set(ament_cmake_copyright_FOUND TRUE)
  # Parse .h headers as C++ rather than C.
  set(ament_cmake_cppcheck_LANGUAGE "c++")
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS LANGUAGE CPP)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
