cmake_minimum_required(VERSION 3.10)
project(mid360_driver)

# 不再硬编码 gcc-13/g++-13（project() 之后的 set 本来也不生效，实测缓存用的是
# 系统默认 /usr/bin/c++）。需要特定编译器请用 toolchain 文件或 -DCMAKE_CXX_COMPILER=。
# -march=native 做成开关：默认保持原行为，但换机/异构部署或与 gtsam_points 的
# 编译选项不一致时可显式关掉。
option(MID360_OPTIMIZE_FOR_NATIVE "Build with -march=native (keep in sync with gtsam_points)" ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${sources})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN mid360_driver::Mid360DriverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CONFIG:Debug>:-O0 -g -Wall -Wextra -Wpedantic -Wconversion -fsanitize=address,undefined -fstack-protector-strong -fcf-protection=full>
  $<$<CONFIG:Release>:-O3 -DNDEBUG -Wall -Wextra -Wpedantic -Wconversion -flto=auto>
)
if(MID360_OPTIMIZE_FOR_NATIVE)
  target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:-march=native>)
endif()

target_link_options(${PROJECT_NAME} PRIVATE
  $<$<CONFIG:Debug>:-fsanitize=address,undefined>
  $<$<CONFIG:Release>:-flto=auto>
)

target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<CONFIG:Debug>:-fsanitize=address,undefined>)
target_link_options(${PROJECT_NAME}_node PRIVATE $<$<CONFIG:Debug>:-fsanitize=address,undefined>)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)