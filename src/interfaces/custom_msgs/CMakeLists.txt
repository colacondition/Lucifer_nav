cmake_minimum_required(VERSION 3.10)
project(decision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowWaypoints.action"
  "msg/RobotStatus.msg"
  "msg/GameStatus.msg"
  "msg/TunnelSpec.msg"
  "msg/SemanticMap.msg"
  "msg/GimbalPosture.msg"
  "msg/GimbalPostureState.msg"
  DEPENDENCIES std_msgs nav_msgs
)

ament_package()
