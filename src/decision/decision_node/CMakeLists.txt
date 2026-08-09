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
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(decision_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(DECISION_LIBRARY_SOURCES
  src/decision_config.cpp
  src/decision_context.cpp
  src/decision_state.cpp
  src/decision_state_machine.cpp
  src/types.cpp
  src/waypoint_executor_client.cpp
  src/waypoint_store.cpp
)

add_library(decision_core ${DECISION_LIBRARY_SOURCES})
target_include_directories(decision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(decision_core
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  decision_interfaces
  ament_index_cpp
)

add_executable(bt_action_replacement_node
  src/decision_node.cpp
)
target_link_libraries(bt_action_replacement_node decision_core)
ament_target_dependencies(bt_action_replacement_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  decision_interfaces
)

install(TARGETS bt_action_replacement_node
  DESTINATION lib/${PROJECT_NAME}
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

  ament_add_gtest(test_decision_state test/test_decision_state.cpp)
  target_link_libraries(test_decision_state decision_core)

  ament_add_gtest(test_decision_state_machine test/test_decision_state_machine.cpp)
  target_link_libraries(test_decision_state_machine decision_core)

  ament_add_gtest(test_decision_config test/test_decision_config.cpp)
  target_link_libraries(test_decision_config decision_core)

  ament_add_gtest(test_waypoint_store test/test_waypoint_store.cpp)
  target_link_libraries(test_waypoint_store decision_core)

  ament_add_gtest(test_waypoint_executor_client test/test_waypoint_executor_client.cpp)
  target_link_libraries(test_waypoint_executor_client decision_core)
endif()

ament_package()
