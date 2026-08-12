cmake_minimum_required(VERSION 3.8)
project(serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(decision_interfaces REQUIRED)

ament_auto_find_build_dependencies()


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_driver_node.cpp
  src/crc.cpp
  src/serial_port.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN serial_driver::SerialDriverNode
  EXECUTABLE ${PROJECT_NAME}_node
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # 锁住云台姿态帧的线上布局。固件那侧要按同样的字节数和偏移解析，改这里必须同步改
  # 固件 —— 这个测试的作用是让「悄悄改了布局」这件事在 CI 上就停下来。
  ament_add_gtest(test_gimbal_posture_packet
    test/test_gimbal_posture_packet.cpp
  )
  target_link_libraries(test_gimbal_posture_packet ${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
