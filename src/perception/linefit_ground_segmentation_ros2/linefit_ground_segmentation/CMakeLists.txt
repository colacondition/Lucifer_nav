cmake_minimum_required(VERSION 3.10)
project(linefit_ground_segmentation)

# find_package(catkin_simple 0.1.0 REQUIRED )
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

find_package(ament_cmake_auto REQUIRED)
# 本机 /lib/x86_64-linux-gnu/cmake/pcl 是 /usr 的 symlink；PCLConfig 用
# PCL_DIR 反推 PCL_ROOT 时会把 /usr 丢掉，clean build 报 "PCL can not be found"。
# 显式给它 /usr 下的配置目录（只影响构建，不改算法）。
if(NOT DEFINED PCL_DIR AND EXISTS "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake")
  set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "PCL config dir")
endif()
find_package(PCL REQUIRED COMPONENTS common visualization)
# 本机 PCL 1.12 的 config 没有把 include 传给 target；find_path 兜底。
find_path(PCL_POINT_CLOUD_INCLUDE_DIR pcl/point_cloud.h)
if(PCL_POINT_CLOUD_INCLUDE_DIR)
  include_directories(SYSTEM ${PCL_POINT_CLOUD_INCLUDE_DIR})
  add_compile_options(-I${PCL_POINT_CLOUD_INCLUDE_DIR})
endif()
find_package(OpenMP REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
find_package(nuc_infra REQUIRED)
# [NUC_infra] 并联算子库
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX nuc_infra::nuc_infra)

if(BUILD_TESTING)
  # 台阶识别的回归测试。链接真实算法跑合成点云，锁住 max_start_height 的取值 ——
  # 这个参数一旦调大，20cm 高台就在代价地图上静默消失，没有任何报错。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_step_detection test/test_step_detection.cpp)
  target_link_libraries(test_step_detection ${PROJECT_NAME})

  # 输入健壮性回归：bin 索引越界 clamp、NaN/Inf 点、r_min/r_max 边界、
  # line_search_angle 邻居线搜索。见文件头注释（2026-10 审计）。
  ament_add_gtest(test_ground_segmentation_robustness
    test/test_ground_segmentation_robustness.cpp)
  target_link_libraries(test_ground_segmentation_robustness ${PROJECT_NAME})
endif()

ament_auto_package()

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})