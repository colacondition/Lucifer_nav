cmake_minimum_required(VERSION 3.10) 
project(linefit_ground_segmentation_ros)

# find_package(catkin_simple 0.1.0 REQUIRED )

# catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

find_package(ament_cmake_auto)

ament_auto_find_build_dependencies()

# Build as a composable component (zero-copy in a shared container) while still
# providing the standalone ground_segmentation_node executable.
ament_auto_add_library(ground_segmentation_component SHARED src/ground_segmentation_node.cc)
rclcpp_components_register_node(ground_segmentation_component
  PLUGIN "linefit_ground_segmentation::SegmentationNode"
  EXECUTABLE ground_segmentation_node)

if(BUILD_TESTING)
  # 锁住 sensor_height 从 TF（URDF 外参）解析这件事。以前这个值靠 yaml 手填，
  # 实车就跟外参对不上了（外参 z=0.0 / yaml 0.59 / xacro default 0.49），
  # 而代价地图的高度带是跟着 TF 走的，两边不一致时地面会整片误判，没有报错。
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sensor_height_from_tf.py TIMEOUT 120)
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
