cmake_minimum_required(VERSION 3.8)
project(decision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(decision_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
# BT 决策骨架用 ROS Humble 自带的 BehaviorTree.CPP v3（3.8.x）。包名是
# behaviortree_cpp_v3；导入 target 名在本机两套安装布局下不一样（见下面的兼容段）。
find_package(behaviortree_cpp_v3 REQUIRED)

# behaviortree_cpp_v3 在本机有两套 CMake config：普通安装（lib/cmake，导出
# BT::behaviortree_cpp_v3）和 ament 安装（share/cmake，导出
# behaviortree_cpp_v3::behaviortree_cpp_v3）。两种都兼容，换镜像/重装不会突然找不到。
if(TARGET BT::behaviortree_cpp_v3)
  set(BEHAVIORTREE_CPP_V3_TARGET BT::behaviortree_cpp_v3)
elseif(TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
  set(BEHAVIORTREE_CPP_V3_TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
else()
  message(FATAL_ERROR "behaviortree_cpp_v3 已找到但没有可用的导入 target")
endif()
message(STATUS "decision: using BehaviorTree.CPP target ${BEHAVIORTREE_CPP_V3_TARGET}")


set(DECISION_LIBRARY_SOURCES
  src/center_patrol.cpp
  src/center_zone.cpp
  src/decision_config.cpp
  src/decision_context.cpp
  src/mission_runtime.cpp
  src/target_input.cpp
  src/types.cpp
  src/waypoint_executor_client.cpp
  src/waypoint_store.cpp
)

add_library(decision_core ${DECISION_LIBRARY_SOURCES})
target_include_directories(decision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(decision_core
  ${BEHAVIORTREE_CPP_V3_TARGET}
)
ament_target_dependencies(decision_core
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  rclcpp_action
  tf2
  tf2_ros
  decision_interfaces
)

# ========== 唯一生产节点：BT 决策（BehaviorTree.CPP v3） ==========
# 旧 FSM 节点（bt_action_replacement_node + DecisionStateMachine）已删除。
# tree XML 见 config/trees/，节点/黑板约定见 include/decision/bt_nodes.hpp +
# mission_runtime.hpp，行为说明见 docs/decision/README.md。
#
add_executable(bt_decision_node
  src/bt_decision_node.cpp
)
target_include_directories(bt_decision_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(bt_decision_node
  decision_core
  ${BEHAVIORTREE_CPP_V3_TARGET}
)
ament_target_dependencies(bt_decision_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  std_msgs
  rclcpp_action
  tf2
  tf2_ros
  decision_interfaces
)

install(TARGETS bt_decision_node
  DESTINATION lib/${PROJECT_NAME}
)

# 兼容旧 launch 的可执行名：bringup/launch/real.launch.py 仍引用
# `bt_action_replacement_node`（该文件属于 bringup 包，不在本任务改动范围）。
# 同一个二进制装两份名字；节点内部名仍是 bt_action_replacement，yaml 参数段不变。
# ⚠️ 不要同时用两个名字启动（会变成两个同名节点互相抢占，运行时会报 ERROR）。
install(PROGRAMS $<TARGET_FILE:bt_decision_node>
  DESTINATION lib/${PROJECT_NAME}
  RENAME bt_action_replacement_node
)

install(
  PROGRAMS
    scripts/test_topics_publisher_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# BT XML 树安装到 share/decision/trees（bt_decision_node 默认用 ament_index 从这里
# 加载 trees/rmul.xml）。上面 install(DIRECTORY config) 会同时把 XML 复制到
# share/decision/config/trees —— 那条是给现有节点读旧 config 用的，trees/ 是 BT
# 栈的稳定入口，两条都保留。
install(DIRECTORY config/trees
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)

  add_test(
    NAME test_decision_launch
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_decision_launch.py
  )

  ament_add_gtest(test_decision_config test/test_decision_config.cpp)
  target_link_libraries(test_decision_config decision_core)

  ament_add_gtest(test_center_patrol test/test_center_patrol.cpp)
  target_link_libraries(test_center_patrol decision_core)

  ament_add_gtest(test_center_zone test/test_center_zone.cpp)
  target_link_libraries(test_center_zone decision_core)

  ament_add_gtest(test_target_input test/test_target_input.cpp)
  target_link_libraries(test_target_input decision_core)

  ament_add_gtest(test_decision_context test/test_decision_context.cpp)
  target_link_libraries(test_decision_context decision_core)

  ament_add_gtest(test_waypoint_store test/test_waypoint_store.cpp)
  target_link_libraries(test_waypoint_store decision_core)

  ament_add_gtest(test_waypoint_executor_client test/test_waypoint_executor_client.cpp)
  target_link_libraries(test_waypoint_executor_client decision_core)

  # BT 场景单测：不需要 ROS 运行时，用假 backend 跑真实 XML + 真实叶子节点，
  # 覆盖回家/回血/LOST/到中心前不追敌/中心追敌与走位切换/抢占不卡死。
  # XML 路径用源码目录注入，不依赖安装目录/CWD。
  ament_add_gtest(test_bt_tree test/test_bt_tree.cpp)
  target_include_directories(test_bt_tree PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_definitions(test_bt_tree PRIVATE
    DECISION_SOURCE_TREE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/trees"
  )
  target_link_libraries(test_bt_tree
    ${BEHAVIORTREE_CPP_V3_TARGET}
  )
endif()

ament_package()
