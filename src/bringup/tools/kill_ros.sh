#!/bin/bash
# 清理本工作区残留的 ROS / Gazebo 进程。
#
# 使用场景：
#   - ros2 launch 被 kill -9 或异常崩溃后，子进程（gzserver、各节点、容器）
#     不会随 launch 退出，成为孤儿继续占着 Gazebo 主端口 / 发布旧话题 /
#     用旧代码发点云，导致下一次启动「位置不对 / 卡住 / 无响应」。
#   - respawn 节点在 launch 被杀后同样会残留（respawn 由 launch 进程驱动，
#     launch 死了就不再重启，但已启动的实例会活下来）。
#
# 用法：bash src/bringup/tools/kill_ros.sh
# 说明：只杀与本工作区相关的进程，不影响系统里其他 ROS 工作区。

set -u

WORKSPACE="/home/cola/Lucifer_nav"

echo "=== 查找残留进程 ==="
# 本工作区 install 下的所有节点/容器 + gazebo + rviz
PIDS=$(ps -eo pid,cmd | grep -E "$WORKSPACE/install|gzserver|gzclient|/opt/ros/humble/lib/rviz2|component_container" \
  | grep -v grep | grep -v "kill_ros.sh" | awk '{print $1}')

if [ -z "$PIDS" ]; then
  echo "没有发现残留进程。"
else
  echo "待清理 PID:"
  ps -o pid,ppid,etime,cmd -p $PIDS | tail -n +2
  kill $PIDS 2>/dev/null
  sleep 2
  kill -9 $PIDS 2>/dev/null
  echo "已发送 SIGTERM（必要时 SIGKILL）。"
fi

# 清理 FastDDS 共享内存残留（被 kill -9 的进程会留下 /dev/shm/fastrtps_port*，
# 下次启动时报 RTPS_TRANSPORT_SHM Error，并可能拖垮 gazebo spawn 服务发现）。
if [ -d /dev/shm ]; then
  STALE=$(ls /dev/shm 2>/dev/null | grep -i "fastrtps_port" | head -20)
  if [ -n "$STALE" ]; then
    echo "清理 /dev/shm 里的 FastDDS 残留段:"
    echo "$STALE" | sed 's/^/  /'
    rm -f /dev/shm/fastrtps_port* 2>/dev/null
  fi
fi

# 确认 Gazebo 主端口已释放
if command -v ss > /dev/null 2>&1; then
  if ss -tlnp 2>/dev/null | grep -q 11345; then
    echo "警告: Gazebo 主端口 11345 仍被占用，请手动排查。"
  else
    echo "Gazebo 主端口 11345 空闲，可以重新 launch。"
  fi
fi
