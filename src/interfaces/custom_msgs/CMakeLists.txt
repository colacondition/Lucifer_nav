cmake_minimum_required(VERSION 3.10)
project(decision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AllRobotHP.msg"
  "msg/DecisionNum.msg"
  "msg/FriendLocation.msg"
  "msg/RobotStatus.msg"
  "msg/GameStatus.msg"
  "msg/RFID.msg"
  "msg/RobotControl.msg"
)

ament_package()
