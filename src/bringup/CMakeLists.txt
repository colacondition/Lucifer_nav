cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  PCD
  rviz
  map
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# 换场地时离线跑的工具，装成可执行方便 ros2 run 之外直接调用。
#   pcd_to_navmap       —— PCD 点云直接转语义地图 msgpack（高度切片，适合地面平整的点云）
#   pgm_to_navmap       —— slam_toolbox/map_saver_cli 存出的 pgm+yaml 转 msgpack（主链）
#   semantic_map_editor —— msgpack 语义地图标注 GUI，人工补围栏、标隧道
install(PROGRAMS
  tools/pcd_to_navmap.py
  tools/pgm_to_navmap.py
  tools/semantic_map_editor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # 本机 ~/.local 里的 anyio 需要 pytest>=7，而系统 pytest 是 6.2.5，插件自动
  # 加载会直接抛 ModuleNotFoundError。ament_cmake_pytest 在 configure 阶段用
  # `python -m pytest --version` 探测可用性，探测一失败它只警告不报错，测试就被
  # 静默跳过了——比失败更难发现。这里在 configure 前后都关掉插件自动加载：
  # set(ENV{}) 影响 configure 期的 execute_process，ENV 参数影响实际跑测试时。
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pcd_to_navmap test/test_pcd_to_navmap.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_pgm_to_navmap test/test_pgm_to_navmap.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
