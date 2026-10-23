#!/usr/bin/env bash
# ============================================================================
# PTP / 时间基准体检。time_sync_monitor 每 10 s 会跑一次本脚本，把输出
# 发到 /dual_lidar/status/ptp；也可以手工跑：
#     bash $(dirname $0)/ptp_status.sh
#
# 输出里刻意保留一行形如 "offset <ns>"，节点的正则就从这行取 PTP 偏移量。
# 本脚本不做任何需要 root 的写操作，允许字段缺失。
# ============================================================================
set -uo pipefail

IFACE="${PTP_IFACE:-enp7s0}"
RUN_DIR="${PTP_RUN_DIR:-/run/dual_lidar_ptp}"

echo "== PTP 体检 $(date '+%F %T') =="
echo "-- 主机时间: $(date +%s.%N)  ($(date '+%F %T.%N'))"

echo "-- 接口: $(ip -br addr show "${IFACE}" 2>/dev/null || echo "找不到 ${IFACE}")"

if command -v ethtool >/dev/null 2>&1; then
  hw="$(ethtool -T "${IFACE}" 2>/dev/null | grep -E 'PTP Hardware Clock' || true)"
  echo "-- ${hw:-PTP Hardware Clock: 不可用（只能软件时间戳）}"
fi

if [ -d /sys/class/ptp ]; then
  echo "-- /sys/class/ptp: $(ls /sys/class/ptp | tr '\n' ' ')"
fi

for name in ptp4l phc2sys; do
  pidfile="${RUN_DIR}/${name}.pid"
  # 注意：不能用 kill -0 判断——root 起的进程对普通用户会返回 EPERM，
  # 导致"明明在跑却显示未运行"。pgrep -F 按 pid 文件匹配，无权限问题。
  if [ -f "${pidfile}" ] && pgrep -F "${pidfile}" >/dev/null 2>&1; then
    echo "-- ${name}: 运行中 (pid $(cat "${pidfile}"))"
  else
    echo "-- ${name}: 未运行"
  fi
done

# 本机 ptp4l 的端口状态与主从偏移
if command -v pmc >/dev/null 2>&1; then
  status="$(pmc -u -b 0 'GET TIME_STATUS_NP' 2>/dev/null || true)"
  if [ -n "${status}" ]; then
    echo "-- pmc TIME_STATUS_NP:"
    echo "${status}" | grep -E 'gmPresent|master_offset|ingress_time|clockClass' || true
    off="$(echo "${status}" | grep -oE 'master_offset[[:space:]]+-?[0-9]+' | head -1 | awk '{print $2}')"
    if [ -n "${off}" ]; then echo "offset ${off}"; else echo "offset n/a"; fi
  else
    echo "-- pmc 无响应（ptp4l 没起，或管理报文被交换机 IGMP snooping 丢了）"
    echo "offset n/a"
  fi
  ports="$(pmc -u -b 0 'GET PORT_DATA_SET' 2>/dev/null || true)"
  if [ -n "${ports}" ]; then
    echo "-- 本机端口状态（SLAVE/MASTER 符合预期吗）:"
    echo "${ports}" | grep -E 'portState|portIdentity' || true
  fi
else
  echo "-- pmc 未安装（sudo apt install linuxptp）"
  echo "offset n/a"
fi

echo "-- 主机与网卡 PHC 的直接对比（需要 root，失败就忽略）:"
if command -v phc_ctl >/dev/null 2>&1; then
  phc_ctl "${IFACE}" get 2>/dev/null | tail -2 || echo "   phc_ctl 需要 root"
fi

echo "== 判据 =="
echo "  offset 绝对值稳定在几十~几百 ns  : PTP 锁上了"
echo "  offset 抖到 µs~ms 或时有时无     : 交换机没做透明时钟，或 IGMP snooping 丢了多播"
echo "  看 time_sync 的 latency_min_ms   : PTP 生效后应在 1 ms 量级且不随时间漂"
