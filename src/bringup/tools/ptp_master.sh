#!/usr/bin/env bash
# ============================================================================
# 主机当 PTP 主钟（grandmaster），两颗 Mid-360 做 slave。
#
#   sudo ./ptp_master.sh start     起 ptp4l + phc2sys（后台，pid 在 /run）
#   sudo ./ptp_master.sh stop      停掉
#   sudo ./ptp_master.sh status    看一眼
#
# 为什么让主机做主钟（而不是雷达做主钟）：
#   雷达做主钟时它们报的是 PTP 纪元时间（TAI 或 GPS 历元），而 ROS/代价地图的
#   "现在"是主机系统时间（UTC），两者差 18~37 秒 —— 代价地图的
#   observationIsFresh（now() 比 stamp）会把点云当"未来时间戳"**静默丢掉**。
#   主机做主钟 + phc2sys 把网卡硬件钟对准 CLOCK_REALTIME，雷达报的就是主机时间，
#   这个坑直接不存在。
#
# 前置条件：
#   1) 网卡支持 PTP 硬件时间戳（ethtool -T enp7s0 看 PTP Hardware Clock）
#      不支持就退化成软件时间戳（-S，精度 ~100 µs 级，配合 min 滤波通常也够）；
#   2) 装了 linuxptp： sudo apt install linuxptp
#   3) 两颗雷达已在 Livox Viewer 2 / SDK 里把时间同步方式设成 PTP 并写入设备
#      （驱动是纯被动收包，不会替你在雷达侧开 PTP）。
# ============================================================================
set -euo pipefail

IFACE="${PTP_IFACE:-enp7s0}"
RUN_DIR="${PTP_RUN_DIR:-/run/dual_lidar_ptp}"
# utc_offset 0：不做闰秒换算，PTP 域的时钟就是主机 UTC 时钟。
# 用 -O 0 显式指定，避免 linuxptp 默认配置里的 utc_offset 37 让链路两端理解不一致。
UTC_OFFSET="${PTP_UTC_OFFSET:-0}"

mkdir -p "${RUN_DIR}"
NTP_STATE_FILE="${RUN_DIR}/ntp_state"

hw_supported() {
  command -v ethtool >/dev/null 2>&1 || return 1
  ethtool -T "${IFACE}" 2>/dev/null | grep -qE 'PTP Hardware Clock: *[0-9]+'
}

# 上游 navi_minco_bit 的实战经验（scripts/ptp_sync.bash）：先停系统对时。
# 原因：phc2sys 把网卡 PHC 锁到 CLOCK_REALTIME，NTP 一旦**步进/回拨**系统时间，
# PHC 跟着往回走，雷达报的时间戳会倒退——录的包时间轴断裂、代价地图把点云判成"未来"。
disable_ntp() {
  local ts chrony
  ts="$(systemctl is-active systemd-timesyncd 2>/dev/null || echo unknown)"
  chrony="$(systemctl is-active chrony 2>/dev/null || echo unknown)"
  echo "systemd-timesyncd=${ts} chrony=${chrony}" > "${NTP_STATE_FILE}"
  timedatectl set-ntp false 2>/dev/null || true
  systemctl stop systemd-timesyncd 2>/dev/null || true
  systemctl stop chrony 2>/dev/null || true
  echo ">>> 已停系统对时（原状态：timesyncd=${ts} chrony=${chrony}；stop 时自动恢复）"
}

restore_ntp() {
  [ -f "${NTP_STATE_FILE}" ] || return 0
  local line
  line="$(cat "${NTP_STATE_FILE}")"
  case "${line}" in *"chrony=active"*)  systemctl start chrony 2>/dev/null || true ;; esac
  case "${line}" in *"timesyncd=active"*) systemctl start systemd-timesyncd 2>/dev/null || true ;; esac
  rm -f "${NTP_STATE_FILE}"
  echo ">>> 已按原状态恢复系统对时"
}

start() {
  if [ -f "${RUN_DIR}/ptp4l.pid" ] && kill -0 "$(cat "${RUN_DIR}/ptp4l.pid")" 2>/dev/null; then
    echo "ptp4l 已在运行 (pid $(cat "${RUN_DIR}/ptp4l.pid"))"
    exit 0
  fi
  local ts_flag="-S"
  local mode="software"
  if hw_supported; then ts_flag="-H"; mode="hardware"; fi
  echo ">>> 网卡 ${IFACE} 使用 ${mode} 时间戳（${ts_flag}）"
  disable_ntp

  # 实时优先级 + 行缓冲：上游实测不加 stdbuf 时日志会被吞掉
  local prefix=()
  command -v stdbuf >/dev/null 2>&1 && prefix+=(stdbuf -oL)
  command -v chrt   >/dev/null 2>&1 && prefix+=(chrt -f 99)

  # 默认 profile = IEEE1588v2 over UDP/IPv4（多播 224.0.1.129），
  # Mid-360 的 PTP 也是 UDP/IP，二者匹配。
  # 注意 `-2`（transportSpecific=802.1AS/gPTP）：上游用的是 -2，但 Mid-360 的
  # PTP 默认走 1588v2 over UDP/IPv4，两者 transportSpecific 必须与从端一致，
  # 不确定就别加。要试：PTP_TS_PROFILE=-2 sudo ./ptp_master.sh start
  # ⚠️ ptp4l 没有 `-O` 短选项（那是 phc2sys 的），utc_offset 只能用长选项/配置文件。
  # 传 `-O 0` 会让 ptp4l 打印 usage 并立刻退出（实测）。
  # 默认用 `-2`（IEEE 802.3 二层 PTP）：实测可用，且不依赖 IGMP（非管理交换机
  # 常把 224.0.1.129 的 IP 多播过滤掉）。要回到 UDP 模式：PTP_TS_PROFILE=-4
  "${prefix[@]}" ptp4l -i "${IFACE}" ${ts_flag} --utc_offset="${UTC_OFFSET}" -m ${PTP_TS_PROFILE:--2} \
    --step_threshold=1.0 --tx_timestamp_timeout=20 \
    >"${RUN_DIR}/ptp4l.log" 2>&1 &
  echo $! > "${RUN_DIR}/ptp4l.pid"

  sleep 2
  if [ "${mode}" = "hardware" ]; then
    # 有 PHC 才需要把硬件钟对准系统时钟；软件时间戳模式下 ptp4l 直接读
    # CLOCK_REALTIME，没有 PHC 可对——此时起 phc2sys 只会报错退出。
    "${prefix[@]}" phc2sys -s CLOCK_REALTIME -c "${IFACE}" -O "${UTC_OFFSET}" -w -m \
      >"${RUN_DIR}/phc2sys.log" 2>&1 &
    echo $! > "${RUN_DIR}/phc2sys.pid"
    sleep 1
  else
    echo ">>> 软件时间戳模式：跳过 phc2sys（没有 PHC 可同步，ptp4l -S 直接用系统时钟当主钟）"
  fi

  echo ">>> 已启动。日志：${RUN_DIR}/{ptp4l,phc2sys}.log"
  echo ">>> 用 ./ptp_status.sh 看状态；上车前务必确认雷达端没有 UTC 偏移（见 README）"
}

stop() {
  for name in phc2sys ptp4l; do
    if [ -f "${RUN_DIR}/${name}.pid" ]; then
      pid="$(cat "${RUN_DIR}/${name}.pid")"
      if kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}" && echo "已停 ${name} (pid ${pid})"
      fi
      rm -f "${RUN_DIR}/${name}.pid"
    fi
  done
  restore_ntp
}

status() {
  bash "$(dirname "$0")/ptp_status.sh"
}

# ---------------------------------------------------------------------------
# 装成 systemd 服务：开机自动起主钟，不用每次手动
#   sudo ./ptp_master.sh install [网口] [hw|sw]
# ---------------------------------------------------------------------------
install_service() {
  local iface="${1:-}"
  local mode="${2:-auto}"
  # 不给网口就**自动挑**：优先有 192.168.1.x 地址的那块，否则第一块 UP 的有线口。
  # 之前手抄成 enp86s0（NUC 的名字）装到开发机上，ptp4l 直接 "No such device" 死循环。
  if [ -z "${iface}" ]; then
    iface="$(ip -4 -br addr 2>/dev/null | awk '$1 !~ /^(lo|docker|veth|br-)/ && $3 ~ /^192\.168\.1\./ {print $1; exit}')"
  fi
  if [ -z "${iface}" ]; then
    iface="$(ip -4 -br addr 2>/dev/null | awk '$1 ~ /^(en|eth)/ && $2 == "UP" {print $1; exit}')"
  fi
  if [ -z "${iface}" ]; then
    echo "!! 找不到有线网口，请显式给：sudo $0 install <网口> [hw|sw]" >&2
    ip -br addr | sed 's/^/    /' >&2
    return 1
  fi
  local ts=""
  if [ "${mode}" = "hw" ]; then
    ts="-H"
  elif [ "${mode}" = "sw" ]; then
    ts="-S"
  else
    IFACE="${iface}"
    if hw_supported; then ts="-H"; else ts="-S"; fi
  fi
  local unit=/etc/systemd/system/ptp-master.service
  echo ">>> 接口 ${iface}，时间戳 ${ts}（auto 时按网卡有没有 PHC 决定）"
  cat > "${unit}" <<UNIT
[Unit]
Description=Livox MID-360 PTP grandmaster (dual lidar time sync)
After=network-online.target
Wants=network-online.target
Conflicts=systemd-timesyncd.service

[Service]
Type=simple
# 先关系统对时：NTP 步进/回拨会让 PTP 时间戳倒退（录包时间轴断裂）
ExecStartPre=-/usr/bin/timedatectl set-ntp false
ExecStart=/usr/sbin/ptp4l -i ${iface} ${ts} -2 --utc_offset=0 -m
Restart=always
RestartSec=2
Nice=-10

[Install]
WantedBy=multi-user.target
UNIT
  systemctl daemon-reload
  systemctl enable --now ptp-master
  sleep 2
  if systemctl is-active --quiet ptp-master; then
    echo ">>> ptp-master 已启动并设为开机自启"
  else
    echo ">>> !! 服务没起来，最近日志：" >&2
    journalctl -u ptp-master -n 6 --no-pager 2>/dev/null | sed 's/^/    /' >&2 \
      || systemctl --no-pager status ptp-master | head -10 >&2
    echo "    最常见原因：单元里的网口名和本机不一致" >&2
  fi
  systemctl --no-pager status ptp-master | head -8
  echo ">>> 以后开机会自动起。验证："
  echo "    systemctl is-enabled ptp-master     # 期望 enabled"
  echo "    $(dirname "$0")/ptp_status.sh"
}

uninstall_service() {
  systemctl disable --now ptp-master 2>/dev/null || true
  rm -f /etc/systemd/system/ptp-master.service
  systemctl daemon-reload
  timedatectl set-ntp true 2>/dev/null || true
  echo ">>> 已卸载 ptp-master.service（系统对时已恢复默认）"
}

case "${1:-start}" in
  start) start ;;
  stop) stop ;;
  restart) stop; sleep 1; start ;;
  status) status ;;
  install) install_service "${2:-}" "${3:-auto}" ;;
  uninstall) uninstall_service ;;
  *) echo "用法: $0 {start|stop|restart|status|install [网口] [hw|sw]|uninstall}"; exit 1 ;;
esac
