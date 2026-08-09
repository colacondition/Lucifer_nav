cmake_minimum_required(VERSION 3.8)
project(cpp_lidar_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED COMPONENTS common io filters)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/filter_node.cpp
  src/radial_filter.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN cpp_lidar_filter::LidarFilterNode
  EXECUTABLE lidar_filter_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_radial_filter test/test_radial_filter.cpp)
  target_include_directories(test_radial_filter PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_radial_filter ${PROJECT_NAME} ${PCL_LIBRARIES})
endif()

ament_auto_package()
