cmake_minimum_required(VERSION 3.8)
project(sentry_waypoints)

find_package(ament_cmake REQUIRED)

# 纯数据包：把 waypoints 目录原样装进 share，供各消费方拼绝对路径。
install(DIRECTORY waypoints DESTINATION share/${PROJECT_NAME})

ament_package()
