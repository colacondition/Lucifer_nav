cmake_minimum_required(VERSION 3.8)
project(fast_location)

# 设置 CMake 策略以避免警告
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========== 查找依赖 ==========
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(CUDA QUIET)

# ========== 包含目录 ==========
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# ========== PCL 配置 ==========
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ========== 可执行文件 ==========
add_library(global_search
  src/global_search.cpp
)
target_include_directories(global_search PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(global_search
  ${PCL_LIBRARIES}
)

add_executable(robot_localization_node
  src/robot_localization.cpp
)

# ========== 链接库 ==========
target_link_libraries(robot_localization_node
  global_search
  ${PCL_LIBRARIES}
)

# 添加 fast_gicp 库
if(fast_gicp_FOUND)
  target_link_libraries(robot_localization_node ${fast_gicp_LIBRARIES})
endif()

# 如果找到CUDA，添加CUDA库
if(CUDA_FOUND)
  target_include_directories(robot_localization_node PRIVATE ${CUDA_INCLUDE_DIRS})
  target_link_libraries(robot_localization_node ${CUDA_LIBRARIES})
  message(STATUS "CUDA found, GPU acceleration enabled")
else()
  message(STATUS "CUDA not found, using CPU-only mode")
endif()

ament_target_dependencies(robot_localization_node
  rclcpp
  ament_index_cpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  pcl_conversions
  fast_gicp
)

# ========== 安装 ==========
install(TARGETS
  robot_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# 导出头文件目录供其他包使用
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(frame_transforms_test
    test/frame_transforms_test.cpp
  )
  target_include_directories(frame_transforms_test PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
  )

  ament_add_gtest(alignment_quality_test
    test/alignment_quality_test.cpp
  )
  target_include_directories(alignment_quality_test PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
  )

  ament_add_gtest(global_search_test
    test/global_search_test.cpp
  )
  target_include_directories(global_search_test PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(global_search_test
    global_search
    ${PCL_LIBRARIES}
  )

  add_test(
    NAME fast_location_config_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fast_location_config_test.py
  )
endif()

ament_package()
