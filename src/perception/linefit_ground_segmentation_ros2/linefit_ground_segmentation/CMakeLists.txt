cmake_minimum_required(VERSION 3.10)
project(linefit_ground_segmentation)

# find_package(catkin_simple 0.1.0 REQUIRED )
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization)
find_package(OpenMP REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

if(BUILD_TESTING)
  # 台阶识别的回归测试。链接真实算法跑合成点云，锁住 max_start_height 的取值 ——
  # 这个参数一旦调大，20cm 高台就在代价地图上静默消失，没有任何报错。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_step_detection test/test_step_detection.cpp)
  target_link_libraries(test_step_detection ${PROJECT_NAME})
endif()

ament_auto_package()

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})