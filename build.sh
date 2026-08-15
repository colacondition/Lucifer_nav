#!/bin/bash
# Lucifer Navigation 编译脚本
# 低负载编译：包间顺序执行（colcon 按 package.xml 依赖自动拓扑排序），
# 包内按核数并行并加 -l 负载限制，避免全量并发把机器压死。
#
# 用法:
#   ./build.sh                      编译全部包
#   ./build.sh navigation2 bringup  只编译指定包（依赖顺序由 colcon 解析）
#   ./build.sh --list               列出 src 下所有包后退出

set -euo pipefail

cd "$(dirname "$0")"

# 环境检查
if [ -z "${ROS_DISTRO:-}" ]; then
  echo "错误: ROS 2 环境未初始化"
  echo "请先执行: source /opt/ros/humble/setup.bash"
  exit 1
fi

if [ "${1:-}" = "--list" ]; then
  colcon list
  exit 0
fi

NPROC=$(nproc)
export CMAKE_BUILD_PARALLEL_LEVEL="$NPROC"
# -l 负载均值限流：机器忙时让出 CPU，比固定 -j1 更快且不压死机器。
export MAKEFLAGS="-j${NPROC} -l${NPROC}"

SELECT_ARGS=()
if [ $# -gt 0 ]; then
  SELECT_ARGS=(--packages-select "$@")
fi

echo "=== Lucifer Navigation Build Script ==="
echo "工作目录: $(pwd)"
echo "ROS 发行版: $ROS_DISTRO"
echo "编译模式: 包间顺序、包内并行 -j${NPROC} -l${NPROC}、低优先级"
if [ ${#SELECT_ARGS[@]} -gt 0 ]; then
  echo "指定包: $*"
else
  echo "编译范围: src 下全部包"
fi
echo ""

# set -e 下失败会直接退出；colcon 的摘要里会标出失败包名。
nice -n 10 ionice -c3 colcon build --symlink-install \
  --executor sequential \
  "${SELECT_ARGS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=== 编译完成 ==="
echo "执行以下命令激活环境:"
echo "  source install/setup.bash"
