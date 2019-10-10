cmake_minimum_required(VERSION 3.10)
project(swri_math_util)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost COMPONENTS random) 
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions( -DBOOST_1_46 )
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

include_directories(include ${Boost_INCLUDE_DIRS})
  
add_library(${PROJECT_NAME}
  src/math_util.cpp
  src/trig_util.cpp
  src/random.cpp
  src/interpolation_1d.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_trig_util test/test_trig_util.cpp)
  target_link_libraries(test_trig_util ${PROJECT_NAME})

  ament_add_gtest(test_math_util test/test_math_util.cpp)
  target_link_libraries(test_math_util ${PROJECT_NAME})

  ament_add_gtest(test_random test/test_random.cpp)
  target_link_libraries(test_random ${PROJECT_NAME})
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
ament_export_dependencies(rclcpp)
ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

ament_package()