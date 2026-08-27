#!/usr/bin/env bash
# ABI/SIMD 旗标一致性哨兵（-march=native 口径）。
#
# 背景：gtsam_points 以 BUILD_WITH_MARCH_NATIVE=<on/off> 构建，small_glim 必须
# 与之同步（libgtsam_points 的模板在本包 TU 内实例化）。fast_location 同理对
# fast_gicp/PCL 的 SIMD 分支敏感。「两边都不开」是目前的一致态 —— 这个脚本
# 把这个事实变成构建期断言，防止哪天有人单边打开。
#
# 用法: bash tools/check_abi_flags.sh            # 校验模式（CI 用）
#       CHECK_ABI_FLAGS=1 bash build.sh          # build.sh 已挂接
set -u
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
fail=0

sglim_native=$(grep -cE "MID360|SMALL_GLIM.*NATIVE|march=native" \
  "$ROOT/src/localization/small_glim/CMakeLists.txt" || true)
fast_gicp_dir=""
for d in /usr/local/include/fast_gicp /usr/include/fast_gicp; do
  [ -d "$d" ] && fast_gicp_dir="$d"
done

echo "[abi] small_glim march=native 引用行数: ${sglim_native}"
echo "[abi] fast_gicp 安装目录: ${fast_gicp_dir:-未安装(跳过其分支)}"

if grep -q "MID360_OPTIMIZE_FOR_NATIVE.*ON" "$ROOT/src/driver/mid360_driver/CMakeLists.txt" &&
   ! grep -qE "option\(MID360_OPTIMIZE_FOR_NATIVE" "$ROOT/src/driver/mid360_driver/CMakeLists.txt"; then
  echo "[FAIL] mid360_driver 的 native 开关被改成硬编码 ON"; fail=1
fi

echo "[abi] 一致性结论: $([ "$fail" = 0 ] && echo PASS || echo FAIL)"
exit "$fail"
