#!/usr/bin/env python3
"""导航链路「感知 -> 新路径」反应延迟基准（无 Gazebo，合成输入）。

流程：
  1. 等 rm_map_server 的 /map（transient_local），在自由区选一条 4~5 m 直线走廊；
  2. 以 50 Hz 发 map->base_link_fake TF、30 Hz 发 /Odometry（map 系、零速度）、
     1 Hz 发 /localization_status=OK；
  3. 发 /goal_pose，等 /plan_raw 成熟（基线路径穿过障碍点）；
  4. 每轮在路径中点注入一堵 2 m 宽的点云墙，测：
       - 注入首帧 -> 第一条绕开障碍的 /plan_raw  (A* 反应)
       - 注入首帧 -> 第一条绕开障碍的 /plan      (+ MINCO)
     然后撤掉障碍，等代价图衰减、路径回到基线，进入下一轮；
  5. 输出 JSON：每条链路的均值/p50/p95，以及代价图侧的反应时间。

用法：
  python3 bench.py --trials 12 --out before.json
"""
import argparse
import json
import os
import random
import math
import statistics
import threading
import time

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster


def make_cloud(points, frame="map"):
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    import struct
    buf = bytearray(msg.row_step)
    for i, (x, y, z) in enumerate(points):
        struct.pack_into("<fff", buf, i * 12, x, y, z)
    msg.data = bytes(buf)
    return msg


class Bench(Node):
    def __init__(self, trials: int):
        super().__init__("nav_reaction_bench")
        self.trials = trials
        self.tf_broadcaster = TransformBroadcaster(self)
        self.map_ = None
        self.plan_raw = None
        self.plan_raw_t = 0.0
        self.plan = None
        self.plan_t = 0.0
        self.costmap = None
        self.costmap_t = 0.0
        self.pose = (0.0, 0.0, 0.0)  # x, y, yaw
        self.moving = False
        self.move_start = (0.0, 0.0, 0.0)
        self.move_t0 = 0.0
        self.speed = 1.0
        self.lock = threading.Lock()

        map_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap, map_qos)
        self.create_subscription(Path, "/plan_raw", self._on_plan_raw, 10)
        self.create_subscription(Path, "/plan", self._on_plan, 10)

        self.pc_pub = self.create_publisher(PointCloud2, "/segmentation/obstacle", 10)
        self.odom_pub = self.create_publisher(Odometry, "/Odometry", 10)
        self.status_pub = self.create_publisher(String, "/localization_status", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.replan_pub = self.create_publisher(Empty, "/navigation2/replan_request", 10)
        self.create_timer(0.02, self._publish_tf)
        self.create_timer(0.033, self._publish_odom)
        self.create_timer(1.0, self._publish_status)

    # ---------- callbacks ----------
    def _on_map(self, msg):
        with self.lock:
            if self.map_ is None:
                self.get_logger().info(
                    f"map: {msg.info.width}x{msg.info.height} "
                    f"res={msg.info.resolution} origin=({msg.info.origin.position.x:.2f},"
                    f"{msg.info.origin.position.y:.2f})")
            self.map_ = msg

    def _on_costmap(self, msg):
        with self.lock:
            self.costmap = msg
            self.costmap_t = time.monotonic()

    def _on_plan_raw(self, msg):
        with self.lock:
            self.plan_raw = msg
            self.plan_raw_t = time.monotonic()

    def _on_plan(self, msg):
        with self.lock:
            self.plan = msg
            self.plan_t = time.monotonic()

    # ---------- periodic ----------
    def _publish_tf(self):
        if self.moving:
            x0, y0, yaw0 = self.move_start
            self.pose = (x0 + self.speed * (time.monotonic() - self.move_t0), y0, yaw0)
        x, y, yaw = self.pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link_fake"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def _publish_odom(self):
        x, y, yaw = self.pose
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.child_frame_id = "base_link_fake"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_pub.publish(m)

    def _publish_status(self):
        self.status_pub.publish(String(data="OK bench"))

    # ---------- helpers ----------
    def set_pose(self, x, y, yaw=0.0):
        self.pose = (x, y, yaw)
        self.move_start = (x, y, yaw)
        self.move_t0 = time.monotonic()

    def start_motion(self):
        self.moving = True
        self.move_t0 = time.monotonic()

    def stop_motion(self):
        self.moving = False

    def publish_goal(self, x, y):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def old_stamp(self):
        return (self.get_clock().now() - rclpy.time.Duration(seconds=0.1)).to_msg()

    def publish_obstacle(self, obs_x, obs_y, half_width=1.0, step=0.05, z=0.5):
        # /segmentation/obstacle 是机器人系（base_link）点云：range 过滤按机器人
        # 距离算，所以墙必须用相对当前位姿的坐标发布。
        rx, ry, _ = self.pose
        pts = []
        n = int(2 * half_width / step)
        for i in range(n + 1):
            pts.append((obs_x - rx, obs_y - ry - half_width + i * step, z))
        cloud = make_cloud(pts, frame="base_link_fake")
        cloud.header.stamp = self.old_stamp()
        self.pc_pub.publish(cloud)

    def path_min_dist(self, msg, x, y):
        best = 1e9
        for p in msg.poses:
            d = math.hypot(p.pose.position.x - x, p.pose.position.y - y)
            best = min(best, d)
        return best

    def wall_max_cost(self, obs_x, obs_y, half_width=1.0):
        vals = []
        n = int(2 * half_width / 0.1)
        for i in range(n + 1):
            c = self.costmap_cell(obs_x, obs_y - half_width + i * 0.1)
            if c is not None:
                vals.append(c)
        return max(vals) if vals else 999

    def costmap_cell(self, wx, wy):
        with self.lock:
            m = self.costmap
        if m is None:
            return None
        info = m.info
        gx = int((wx - info.origin.position.x) / info.resolution)
        gy = int((wy - info.origin.position.y) / info.resolution)
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return None
        return m.data[gy * info.width + gx]

    def free_segment(self, target_len=4.5, clearance_want=1.2):
        """在 /map 里找一段水平自由走廊，返回 (start_x, start_y, goal_x, goal_y)。"""
        m = self.map_
        info = m.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = m.data

        def free(cx, cy):
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                return False
            v = data[cy * w + cx]
            return 0 <= v < 50

        best = None
        for cy in range(4, h - 4):
            cx = 4
            while cx < w - 4:
                if not free(cx, cy):
                    cx += 1
                    continue
                run_start = cx
                while cx < w - 4 and free(cx, cy):
                    cx += 1
                run_end = cx - 1
                if (run_end - run_start) * res >= target_len:
                    # 走廊中线，用上下各 1.2 m 的净空做评分
                    for frac in (0.35, 0.5, 0.65):
                        sx = run_start + int((run_end - run_start) * frac)
                        clear = 1e9
                        for dy in range(-int(clearance_want / res),
                                        int(clearance_want / res) + 1):
                            for dx in range(-6, int(target_len / res) + 7):
                                if not free(sx + dx, cy + dy):
                                    clear = min(clear, abs(dy) * res)
                        if clear > 1e5:
                            continue
                        cand = (clear, sx, cy)
                        if best is None or cand[0] > best[0]:
                            best = cand
        if best is None:
            raise RuntimeError("no suitable free corridor found")
        clear, sx, cy = best
        start_x = ox + (sx + 0.5) * res
        start_y = oy + (cy + 0.5) * res
        goal_x = start_x + target_len
        self.get_logger().info(
            f"corridor: start=({start_x:.2f},{start_y:.2f}) "
            f"goal=({goal_x:.2f},{start_y:.2f}) clearance>={clear:.2f} m")
        return start_x, start_y, goal_x, start_y

    def wait_until(self, pred, timeout, period=0.02):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if pred():
                return True
            time.sleep(period)
        return False


def find_pid(pattern):
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if pattern in cmd:
            return int(pid)
    return None


def cpu_ticks(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            fields = f.read().rsplit(')', 1)[1].split()
        return int(fields[11]) + int(fields[12])
    except (OSError, IndexError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--out", default="/tmp/nav_reaction/result.json")
    ap.add_argument("--obs-offset", type=float, default=2.2)
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Bench(args.trials)
    import threading as _th
    _th.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    log = lambda s: node.get_logger().info(s)
    if not node.wait_until(lambda: node.map_ is not None, 20.0):
        raise SystemExit("no /map received")
    sx, sy, gx, gy = node.free_segment()
    node.set_pose(sx, sy, 0.0)
    node.publish_goal(gx, gy)
    log(f"goal published ({gx:.2f},{gy:.2f})")

    if not node.wait_until(lambda: node.plan_raw is not None, 20.0):
        raise SystemExit("no /plan_raw received")
    for _ in range(3):
        node.replan_pub.publish(Empty())
        time.sleep(0.15)

    obs_x = sx + args.obs_offset
    obs_y = sy
    log(f"obstacle at ({obs_x:.2f},{obs_y:.2f})")

    def plan_avoids(msg=None):
        m = node.plan_raw if msg is None else msg
        return m is not None and node.path_min_dist(m, obs_x, obs_y) > 0.8

    # 基线：路径应穿过障碍点
    node.wait_until(lambda: node.plan_raw is not None
                    and node.path_min_dist(node.plan_raw, obs_x, obs_y) < 0.25, 10.0)

    base_wall_max = node.wall_max_cost(obs_x, obs_y)
    log(f"baseline wall max cost at obstacle segment = {base_wall_max}")
    results = []
    nav_pid = find_pid('nav_container_mt')
    t_run0 = time.monotonic()
    c0 = cpu_ticks(nav_pid) if nav_pid else None
    for trial in range(args.trials):
        # 撤掉障碍（空云 10 Hz 持续 1.5 s），等整段墙都清掉、路径回到基线
        node.stop_motion()
        node.set_pose(sx, sy, 0.0)
        t_clear = time.monotonic()
        while time.monotonic() - t_clear < 1.5:
            empty = make_cloud([], frame="base_link_fake")
            empty.header.stamp = node.old_stamp()
            node.pc_pub.publish(empty)
            time.sleep(0.1)
        ok = node.wait_until(
            lambda: node.wall_max_cost(obs_x, obs_y) <= base_wall_max + 5, 8.0)
        if not ok:
            log(f"trial {trial}: costmap did not clear (max={node.wall_max_cost(obs_x, obs_y)}), skip")
            continue
        # 重发一次目标：清掉 planner 的 stitching 状态，保证每轮从干净基线开始
        node.publish_goal(gx, gy)
        time.sleep(0.2)
        for _ in range(3):
            node.replan_pub.publish(Empty())
            time.sleep(0.15)
        ok = node.wait_until(lambda: node.plan_raw is not None
                             and node.path_min_dist(node.plan_raw, obs_x, obs_y) < 0.15, 8.0)
        if not ok:
            log(f"trial {trial}: baseline path did not come back, skip")
            continue
        node.set_pose(sx, sy, 0.0)
        node.speed = args.speed
        node.start_motion()
        time.sleep(random.uniform(0.15, 0.35))  # 随机化相位，避免与 8 Hz 定时器同拍
        t0 = time.monotonic()
        plan_raw_lat = None
        plan_lat = None
        seen_costmap = None
        k = 0
        while time.monotonic() - t0 < 3.0:
            if k % 5 == 0:  # 10 Hz 注入，20 ms 轮询检测
                node.publish_obstacle(obs_x, obs_y)
            k += 1
            time.sleep(0.02)
            with node.lock:
                pr, prt = node.plan_raw, node.plan_raw_t
                p, pt = node.plan, node.plan_t
            if seen_costmap is None:
                c = node.costmap_cell(obs_x, obs_y)
                if c is not None and c >= 50:
                    seen_costmap = time.monotonic() - t0
            if seen_costmap is not None:
                if plan_raw_lat is None and pr is not None and prt >= t0 \
                        and node.path_min_dist(pr, obs_x, obs_y) > 0.8:
                    plan_raw_lat = prt - t0
                if plan_lat is None and p is not None and pt >= t0 \
                        and node.path_min_dist(p, obs_x, obs_y) > 0.8:
                    plan_lat = pt - t0
            if plan_raw_lat is not None and plan_lat is not None:
                break
        node.stop_motion()
        results.append({
            "trial": trial,
            "costmap_ms": None if seen_costmap is None else 1000 * seen_costmap,
            "plan_raw_ms": None if plan_raw_lat is None else 1000 * plan_raw_lat,
            "plan_ms": None if plan_lat is None else 1000 * plan_lat,
        })
        log(f"trial {trial}: {results[-1]}")

    t_run1 = time.monotonic()
    c1 = cpu_ticks(nav_pid) if nav_pid else None
    cpu_percent = None
    if c0 is not None and c1 is not None:
        hz = os.sysconf('SC_CLK_TCK')
        cpu_percent = round(100.0 * (c1 - c0) / hz / max(1e-6, t_run1 - t_run0), 1)

    def stats(key):
        xs = [r[key] for r in results if r[key] is not None]
        if not xs:
            return {"n": 0}
        xs.sort()
        return {
            "n": len(xs),
            "mean_ms": round(statistics.mean(xs), 1),
            "p50_ms": round(xs[len(xs) // 2], 1),
            "p95_ms": round(xs[min(len(xs) - 1, int(0.95 * len(xs)))], 1),
            "max_ms": round(max(xs), 1),
        }

    out = {
        "trials": len(results),
        "nav_pid": nav_pid,
        "cpu_percent_nav_container": cpu_percent,
        "obstacle": [obs_x, obs_y],
        "start": [sx, sy],
        "goal": [gx, gy],
        "stats": {
            "costmap": stats("costmap_ms"),
            "plan_raw": stats("plan_raw_ms"),
            "plan": stats("plan_ms"),
        },
        "raw": results,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    log("RESULT " + json.dumps(out["stats"]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
