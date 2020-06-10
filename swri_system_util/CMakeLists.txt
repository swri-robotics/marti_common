cmake_minimum_required(VERSION 3.10)
project(swri_system_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem regex)

add_library(${PROJECT_NAME}
  src/file_util.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  Boost::filesystem
  Boost::regex
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  include_directories(${ament_index_cpp_INCLUDE_DIRS})
  ament_add_gtest(test_file_util test/test_file_util.cpp)
  ament_target_dependencies(test_file_util ament_index_cpp)
  target_link_libraries(test_file_util ${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include ${Boost_INCLUDE_DIRECTORIES})
ament_export_libraries(${PROJECT_NAME})

ament_package()
