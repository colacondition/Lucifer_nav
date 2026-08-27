#!/bin/bash
# Lucifer Navigation 编译脚本
# 低负载编译：包间顺序执行（colcon 按 package.xml 依赖自动拓扑排序），
# 包内按核数并行并加 -l 负载限制，避免全量并发把机器压死。
#
# 用法:
#   ./build.sh                      编译全部包
#   ./build.sh navigation2 bringup  只编译指定包（依赖顺序由 colcon 解析）
#   ./build.sh --list               列出编译列表后退出
#   sh build.sh                     也可以（dash 不支持 pipefail，会转到 bash）

# Ubuntu 的 /bin/sh 是 dash；被 `sh build.sh` 调起时先转到 bash。
if [ -z "${BASH_VERSION:-}" ]; then
  exec bash "$0" "$@"
fi

set -euo pipefail

cd "$(dirname "$0")"

# 编译列表。新增包时记得同步，否则 --list / 全量编译的覆盖检查会警告。
PACKAGES=(
  "decision_interfaces"
  "sentry_waypoints"
  "mid360_driver"
  "small_glim"
  "fast_location"
  "cpp_lidar_filter"
  "linefit_ground_segmentation"
  "linefit_ground_segmentation_ros"
  "pointcloud_to_laserscan"
  "navigation2"
  "fake_vel_transform"
  "waypoint_editor"
  "serial_driver"
  "pb_rm_simulation"
  "ros2_livox_simulation"
  "simulated_gimbal"
  "bringup"
  "decision"
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

# 环境检查
if [ -z "${ROS_DISTRO:-}" ]; then
  echo "错误: ROS 2 环境未初始化"
  echo "请先执行: source /opt/ros/humble/setup.bash"
  exit 1
fi

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

NPROC=$(nproc)
export CMAKE_BUILD_PARALLEL_LEVEL="$NPROC"
# -l 负载均值限流：机器忙时让出 CPU，比固定 -j1 更快且不压死机器。
export MAKEFLAGS="-j${NPROC} -l${NPROC}"

echo "=== Lucifer Navigation Build Script ==="
echo "工作目录: $(pwd)"
echo "ROS 发行版: $ROS_DISTRO"
echo "编译模式: 包间顺序、包内并行 -j${NPROC} -l${NPROC}、低优先级"
echo "待编译: ${#BUILD_LIST[@]} 个包: ${BUILD_LIST[*]}"
echo ""

# ABI 哨兵：-march=native 口径的一致性是环境记忆而非编译期约束，
# 每次构建前跑一次断言（CHECK_ABI_FLAGS=0 可显式跳过）。
if [ "${CHECK_ABI_FLAGS:-1}" = "1" ]; then
  bash "$PWD/src/bringup/tools/check_abi_flags.sh" || {
    echo "ABI 旗标断言失败：先解决 march=native 一致性再构建。"
    exit 1
  }
fi

# 参数契约静态校验（与编译无关的 config 关系漂移）。
python3 "$PWD/src/bringup/tools/check_nav_contract.py" "$PWD" || {
  echo "导航参数契约校验失败：见上方 [FAIL] 行。"
  exit 1
}

# set -e 下失败会直接退出；colcon 的摘要里会标出失败包名。
nice -n 10 ionice -c3 colcon build --symlink-install \
  --executor sequential \
  --packages-select "${BUILD_LIST[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=== 编译完成 ==="
echo "执行以下命令激活环境:"
echo "  source install/setup.bash"
