cmake_minimum_required(VERSION 3.10)
project(navigation2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# AddressSanitizer + UndefinedBehaviorSanitizer in Debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
    message(STATUS "AddressSanitizer + UBSan enabled for Debug build")
  endif()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

# OSQP C library (via ros-humble-osqp-vendor) for the MPC controller.
find_package(osqp REQUIRED)

# msgpack (libmsgpack-dev) for the semantic map file format. Header-only C++ API,
# so there is no library to link — just the include path.
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp REQUIRED)

# One shared library from every source in the tree.
# NOTE: file(GLOB) does not re-run when files are added/removed; re-run cmake
# (touch this file or clean-build) after adding a new source.
file(GLOB_RECURSE NAV2_SOURCES CONFIGURE_DEPENDS src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${NAV2_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${MSGPACK_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  osqp::osqp
)

# Register each node as a composable component and keep the standalone
# executable names. The nodes all live in the single shared library above.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmMapServer"        EXECUTABLE rm_map_server_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmGlobalPlanner"    EXECUTABLE rm_global_planner_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmGlobalCostmap"    EXECUTABLE rm_global_costmap_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmMincoPathSmoother" EXECUTABLE rm_minco_path_smoother_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmLocalCostmap"     EXECUTABLE rm_local_costmap_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmMpcController"    EXECUTABLE rm_mpc_controller_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmVelocitySmoother" EXECUTABLE rm_velocity_smoother_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmNav2Compat"       EXECUTABLE rm_nav2_compat_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmTunnelPosture"    EXECUTABLE rm_tunnel_posture_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmGimbalVisualizer" EXECUTABLE rm_gimbal_visualizer_node)

# 固定线程数组件容器：Humble 的 component_container_mt 无法配置 executor
# 线程数（恒为 hardware_concurrency），核多的机器会起一大把空转线程。
# 第一个位置参数指定线程数（默认 2），launch 里 arguments=['2']。
add_executable(nav_container_mt executable/container_fixed_main.cpp)
target_link_libraries(nav_container_mt
  rclcpp::rclcpp
  rclcpp_components::component_manager
)
install(TARGETS nav_container_mt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Le style de code du projet ne suit pas uncrustify ; les 22 fichiers
  # non conformes existaient avant ces modifications.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_local_path_safety
    test/test_local_path_safety.cpp
  )
  target_link_libraries(test_local_path_safety ${PROJECT_NAME})

  ament_add_gtest(test_semantic_map
    test/test_semantic_map.cpp
  )
  target_link_libraries(test_semantic_map ${PROJECT_NAME})

  ament_add_gtest(test_semantic_map_consumer
    test/test_semantic_map_consumer.cpp
  )
  target_link_libraries(test_semantic_map_consumer ${PROJECT_NAME})

  ament_add_gtest(test_tunnel_posture
    test/test_tunnel_posture.cpp
  )
  target_link_libraries(test_tunnel_posture ${PROJECT_NAME})

  ament_add_gtest(test_path_stitching
    test/test_path_stitching.cpp
  )
  target_link_libraries(test_path_stitching ${PROJECT_NAME})

  ament_add_gtest(test_route_tracker
    test/test_route_tracker.cpp
  )
  target_link_libraries(test_route_tracker ${PROJECT_NAME})

  ament_add_gtest(test_progress_monitor
    test/test_progress_monitor.cpp
  )
  target_link_libraries(test_progress_monitor ${PROJECT_NAME})

  ament_add_gtest(test_recovery_planner
    test/test_recovery_planner.cpp
  )
  target_link_libraries(test_recovery_planner ${PROJECT_NAME})

  ament_add_gtest(test_speed_profile
    test/test_speed_profile.cpp
  )
  target_link_libraries(test_speed_profile ${PROJECT_NAME})

  # ROS 级集成测试：启真节点、喂合成话题、断言 FSM 行为。
  # gtest 只能覆盖纯函数；FSM 状态转移、接近段开关时序、
  # last_published_speed 是否真的驱动 ProgressMonitor 这些必须在节点层验证 ——
  # 「到点蠕动」那个 bug 正是 gtest 全过、只在节点跑起来才暴露的一类。
  # 必须用 add_launch_test 而不是 ament_add_pytest_test：这两个文件是 launch
  # test，依赖 launch_testing 的 pytest 插件解析 generate_test_description。
  # 用 ament_add_pytest_test + PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 会让插件不加载，
  # 表现为 fixture 'mpc' not found。
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_near_goal_no_recovery.py TIMEOUT 120)
  add_launch_test(test/test_stuck_enters_recovery.py TIMEOUT 120)
  add_launch_test(test/test_veto_escalates.py TIMEOUT 60)
  add_launch_test(test/test_hazard_recovery_cycle.py TIMEOUT 120)
  # 锁住障碍高度带是「相对机器人底盘」的差值（point.z - robo_z）这件事。改成任何
  # 一种绝对判法，20cm 高台都会从局部代价地图上静默消失、没有任何报错：在
  # transformPoint 之前比原始 z 是 +0.025，在 map 里跟固定 z=0 比也只有 +0.025
  # （map 的 z=0 是雷达平面，不是地面）。
  add_launch_test(test/test_costmap_height_frame.py TIMEOUT 120)
  # 锁住隧道影响区（本体 + tunnel_margin_m 边距）内的点云不进代价地图、区外的照进。
  # 顶板、门楣和墙在点云里没有区别，按常规高度带判定会把整个洞口涂成致命格，规划器
  # 彻底看不到通路 —— 无报错，表现只是「车不走那条唯一的路」。反向也测边距的边界：
  # 放行泄漏出影响区等于把高度上限整个调没了。
  add_launch_test(test/test_costmap_tunnel_roof.py TIMEOUT 120)
  # 锁住收云台请求在节点层是通的。判定本身有 gtest，这里管接线：语义地图的 QoS、
  # tf 的坐标系、话题名。这三处错的表现完全一样 —— 标志位永远 false，日志里没有报错，
  # 云台在洞口撞上去。
  add_launch_test(test/test_tunnel_posture_node.py TIMEOUT 120)
  # 锁住「等云台收到位再走」。判据用了三条话题上的值，gtest 碰不到；错了的表现是车照常
  # 开进去、云台撞在顶板上，没有任何一层会报错。也一并回归「没有云台请求时不影响行驶」——
  # 这个门在 control() 的主路径上，写反会把全部导航停掉。
  add_launch_test(test/test_gimbal_gate.py TIMEOUT 90)
  # 「一条回传都没收到过」必须单独一个进程：/gimbal_posture_state 是 transient_local，
  # 跟别的用例共进程的话会被前面发过的姿态污染，这个分支根本进不去。
  add_launch_test(test/test_gimbal_gate_no_feedback.py TIMEOUT 90)
  # 锁住「LOST 停车、非 LOST 不停」。gtest 碰不到订阅+control() 调度。
  # 写反的表现是丢定位后仍跟 map 系路径。
  add_launch_test(test/test_integrity_gate.py TIMEOUT 90)
  # 「从没收到过 localization_status」必须单独进程：话题是 transient_local，
  # 跟 LOST 用例共进程会被前面 latch 污染，缺省分支进不去。
  add_launch_test(test/test_integrity_gate_no_status.py TIMEOUT 90)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  params
)
