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
  PLUGIN "navigation2::RmPathSmoother"     EXECUTABLE rm_path_smoother_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmLocalCostmap"     EXECUTABLE rm_local_costmap_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmMpcController"    EXECUTABLE rm_mpc_controller_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmVelocitySmoother" EXECUTABLE rm_velocity_smoother_node)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "navigation2::RmNav2Compat"       EXECUTABLE rm_nav2_compat_node)

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

  ament_add_gtest(test_distance_field
    test/test_distance_field.cpp
  )
  target_link_libraries(test_distance_field ${PROJECT_NAME})

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
  # gtest 只能覆盖纯函数；FSM 状态转移、goal_approach_controller 开关时序、
  # executed_speed 是否真的驱动 ProgressMonitor 这些必须在节点层验证 ——
  # 「到点蠕动」那个 bug 正是 gtest 全过、只在节点跑起来才暴露的一类。
  # 必须用 add_launch_test 而不是 ament_add_pytest_test：这两个文件是 launch
  # test，依赖 launch_testing 的 pytest 插件解析 generate_test_description。
  # 用 ament_add_pytest_test + PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 会让插件不加载，
  # 表现为 fixture 'mpc' not found。
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_near_goal_no_recovery.py TIMEOUT 120)
  add_launch_test(test/test_stuck_enters_recovery.py TIMEOUT 120)
  add_launch_test(test/test_override_detection.py TIMEOUT 60)
  add_launch_test(test/test_veto_escalates.py TIMEOUT 60)
  add_launch_test(test/test_hazard_recovery_cycle.py TIMEOUT 120)
  # 锁住障碍高度带是「相对机器人底盘」的差值（point.z - robo_z）这件事。改成任何
  # 一种绝对判法，20cm 高台都会从局部代价地图上静默消失、没有任何报错：在
  # transformPoint 之前比原始 z 是 +0.025，在 map 里跟固定 z=0 比也只有 +0.025
  # （map 的 z=0 是雷达平面，不是地面）。
  add_launch_test(test/test_costmap_height_frame.py TIMEOUT 120)
  # 锁住 RC-ESDF 是真的在跑。障碍点云喂一个挡路的点，esdfPathSafe 返回 false，
  # MPC 进恢复。反向测试：注掉 esdf_map_.query 那行，测试必须失败。
  add_launch_test(test/test_esdf_obstacle_cloud.py TIMEOUT 90)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  params
)
