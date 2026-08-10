#!/bin/bash
# Lucifer Navigation 编译脚本
# 低负载顺序编译，避免卡死
#
# 用法:
#   ./build.sh                      编译全部包
#   ./build.sh navigation2 bringup  只编译指定的包（仍按下面的依赖顺序）
#   ./build.sh --list               列出编译顺序后退出

set -euo pipefail

cd "$(dirname "$0")"

# 编译顺序（按依赖关系）。新增功能包时记得同步这里，
# 否则 --list 的一致性检查会报出来。
PACKAGES=(
  "decision_interfaces"
  "livox_ros_driver2"   # 实车不再启动它的节点，但 ros2_livox_simulation 编译需要其 CustomMsg
  "mid360_driver"
  "small_glim"
  "fast_location"
  "cpp_lidar_filter"
  "linefit_ground_segmentation"
  "linefit_ground_segmentation_ros"
  "navigation2"
  "goal_approach_controller"
  "fake_vel_transform"
  "waypoint_editor"
  "serial_driver"
  "decision"
  "pb_rm_simulation"
  "ros2_livox_simulation"
  "bringup"
)

# 检查 src 下是否有没写进上面列表的包
check_coverage() {
  local missing=()
  while read -r name; do
    local found=0
    for pkg in "${PACKAGES[@]}"; do
      [ "$pkg" = "$name" ] && found=1 && break
    done
    [ $found -eq 0 ] && missing+=("$name")
  done < <(find src -name package.xml -not -path '*/build/*' \
             -exec grep -ohPm1 '(?<=<name>)[^<]+' {} \; | sort)

  if [ ${#missing[@]} -gt 0 ]; then
    echo "警告: 以下包在 src/ 里但没写进 build.sh 的 PACKAGES，不会被编译:"
    printf '  - %s\n' "${missing[@]}"
    echo ""
  fi
}

if [ "${1:-}" = "--list" ]; then
  printf '%s\n' "${PACKAGES[@]}"
  echo ""
  check_coverage
  exit 0
fi

echo "=== Lucifer Navigation Build Script ==="
echo "工作目录: $(pwd)"
echo ""

# 环境检查
if [ -z "${ROS_DISTRO:-}" ]; then
  echo "错误: ROS 2 环境未初始化"
  echo "请先执行: source /opt/ros/humble/setup.bash"
  exit 1
fi

# 命令行给了包名就只编这些，但顺序仍按 PACKAGES 走（避免手写顺序踩依赖）
BUILD_LIST=()
if [ $# -gt 0 ]; then
  for want in "$@"; do
    found=0
    for pkg in "${PACKAGES[@]}"; do
      [ "$pkg" = "$want" ] && found=1 && break
    done
    if [ $found -eq 0 ]; then
      echo "错误: 未知的包名 '$want'（用 ./build.sh --list 看可选值）"
      exit 1
    fi
  done
  for pkg in "${PACKAGES[@]}"; do
    for want in "$@"; do
      [ "$pkg" = "$want" ] && BUILD_LIST+=("$pkg") && break
    done
  done
else
  BUILD_LIST=("${PACKAGES[@]}")
  check_coverage
fi

echo "ROS 发行版: $ROS_DISTRO"
echo "编译模式: 顺序、单线程、低优先级"
echo "待编译: ${#BUILD_LIST[@]} 个包"
echo ""

total=${#BUILD_LIST[@]}
index=0
for pkg in "${BUILD_LIST[@]}"; do
  index=$((index + 1))
  echo ">>> [$index/$total] 编译 $pkg"
  # set -e 下失败会直接退出，这里显式 trap 一下好给出包名
  if ! MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 nice -n 10 ionice -c3 \
       colcon build --symlink-install --executor sequential \
       --packages-select "$pkg" \
       --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    echo ""
    echo "错误: $pkg 编译失败"
    echo "日志: log/latest_build/$pkg/stdout_stderr.log"
    exit 1
  fi
  echo ""
done

echo "=== 编译完成 ==="
echo "执行以下命令激活环境:"
echo "  source install/setup.bash"
