#!/usr/bin/env bash
# 一键采集仿真导航运行时指标 —— 在【已启动 sim+nav 的同一终端】里执行：
#   cd ~/Lucifer_nav && bash .run_tools/collect_all.sh
# 总耗时约 75s；结果写入 .run_metrics/（该目录与审查会话共享，跑完说一声即可）。
# 注意：不能开 set -u —— ament 的 setup.bash 会引用未定义变量而报错。
SELF_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SELF_DIR/../install/setup.bash" 2>/dev/null || source install/setup.bash
export ROS_HOME="${PWD}/.ros_home"
OUT="$PWD/.run_metrics"; mkdir -p "$OUT"
cd "$PWD"

echo "[1/6] 节点与话题清单"
timeout 10 ros2 node list | sort > "$OUT/nodes.txt" 2>/dev/null
timeout 8 ros2 topic list | sort > "$OUT/topics.txt" 2>/dev/null

echo "[2/6] /clock、定位状态与仿真实时率 RTF"
(timeout 4 ros2 topic echo --once /clock > "$OUT/clock.txt") 2>/dev/null || echo NO_CLOCK >> "$OUT/clock.txt"
# RTF = Δsim/Δwall：<1 时所有「传感器驱动」链路的墙钟频率同比缩水，
# 是判读 hz_*.txt 的先决条件（10Hz 雷达在 RTF=0.5 下就是 5Hz）。
python3 - "$OUT/rtf.txt" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
rclpy.init(); n=Node('rtf_probe'); q=[]
n.create_subscription(TimeMsg,'/clock',lambda m:q.append((time.monotonic(),m.sec+m.nanosec*1e-9)),10)
dl=time.time()+4
while time.time()<dl and len(q)<2: rclpy.spin_once(n,timeout_sec=0.2)
if len(q)>=2:
    (w0,s0),(w1,s1)=q[0],q[-1]
    open(sys.argv[1],'w').write(f"RTF={(s1-s0)/(w1-w0):.3f}\n")
else:
    open(sys.argv[1],'w').write("RTF=NA\n")
n.destroy_node(); rclpy.shutdown()
PY
(timeout 4 ros2 topic echo --once /localization_status > "$OUT/localization_status.txt") 2>/dev/null

echo "[3/6] 空闲基线 CPU（8s，尚未发目标）"
TAG=idle_pre SAMPLE_SEC=8 bash "$SELF_DIR/metrics.sh"

echo "[4/6] 发导航目标并进入运动负载采样"
# 目标源：用户验证过的巡逻 CSV（pose_x,pose_y 列），逐点尝试直到产出 /plan。
plan_exists() {
  timeout 3 ros2 topic echo --once --field header /plan >/dev/null 2>&1
}
try_goal() {
  local GOAL="{header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {w: 1.0}}}"
  ros2 topic pub --once --keep-alive 2 /goal_pose geometry_msgs/msg/PoseStamped "$GOAL" >/dev/null 2>&1
  sleep 3; plan_exists
}
CSV="src/bringup/config/waypoints/RMUL/patrol.csv"
[ -f "$CSV" ] || CSV="$PWD/src/bringup/config/waypoints/RMUL/patrol.csv"
mapfile -t WPTS < <(awk -F, 'NR>1 && $2!="" && $3!="" {print $2" "$3}' "$CSV" | head -3)
GOT_PLAN=0
for i in "${!WPTS[@]}"; do
  set -- ${WPTS[$i]}
  echo "[collect] 尝试目标$((i+1)): ($1, $2)"
  if try_goal "$1" "$2"; then GOT_PLAN=1; break; fi
done
if [ "$GOT_PLAN" = 1 ]; then
  echo "[collect] 已获得 /plan，进入带载采样"
else
  echo "[collect][WARN] CSV 前三点都未产出 /plan（规划器或起点异常，请查 launch 终端 WARN）" \
       | tee "$OUT/goal_warning.txt"
fi
sleep 2
TAG=active SAMPLE_SEC=12 bash "$SELF_DIR/metrics.sh"

echo "    关键话题频率（各约 9s）"
for t in /Odometry /segmentation/obstacle /local_costmap/costmap /plan /predict_path /cmd_vel_nav; do
  timeout 9 ros2 topic hz --window 40 "$t" > "$OUT/hz_${t//\//_}.txt" 2>&1
done

echo "[5/6] 端到端延迟 + 线程级 top（10s 同窗）"
python3 - "$OUT" <<'PY'
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = rclpy.node.Node('metrics_probe')
node.set_parameters([__import__('rclpy.parameter', fromlist=['Parameter']).Parameter(
    'use_sim_time', value=True)])
out = sys.argv[1]
stats = {}
def mk(name, typ, topic, qos):
    node.create_subscription(typ, topic,
        lambda m, name=name: stats.setdefault(name, []).append(
            (node.get_clock().now().nanoseconds,
             m.header.stamp.sec*10**9 + m.header.stamp.nanosec)), qos)

mk('odom_age',       Odometry,     '/Odometry',              qos_profile_sensor_data)
mk('obstacle_age',   PointCloud2,  '/segmentation/obstacle', qos_profile_sensor_data)
latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
mk('lcostmap_age',   OccupancyGrid,'/local_costmap/costmap', latch)
mk('plan',           Path,         '/plan',                  latch)

deadline = time.time() + 10
while time.time() < deadline and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.2)

with open(f'{out}/e2e_latency.txt', 'w') as f:
    for k, arr in stats.items():
        if len(arr) < 3:
            f.write(f"{k}: n={len(arr)} (样本不足)\n"); continue
        ages = sorted((tr-th)/1e9 for tr, th in arr)
        rate = (len(arr)-1) / ((arr[-1][0]-arr[0][0]) / 1e9) if arr[-1][0] != arr[0][0] else 0.0
        f.write(f"{k}: n={len(arr)} rate={rate:.2f}Hz "
                f"age_mean={sum(ages)/len(ages)*1000:.1f}ms "
                f"age_p50={ages[len(ages)//2]*1000:.1f}ms age_max={ages[-1]*1000:.1f}ms\n")
node.destroy_node(); rclpy.shutdown()
PY
top -b -H -n1 -o %CPU | head -28 > "$OUT/threads_top.txt"

echo "[6/6] 抵达后回落基线（停 8s 再采 8s）"
sleep 8
TAG=idle_post SAMPLE_SEC=8 bash "$SELF_DIR/metrics.sh"

echo ALL_DONE
