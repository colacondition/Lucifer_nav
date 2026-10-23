#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MINCO / navigation end-to-end benchmark (rclpy, no Gazebo).

用途
====
量化「发一个目标 -> 拿到新 /plan」的端到端延迟与路径长度，给
``repair.time_budget_ms`` 的取值提供实车/仿真数据（见 MINCO_README.md 的
「墙钟预算与性能基准」）。手动用 RViz 点目标只能看个大概，这里把每次试验的
延迟、路径长度、成功/超时逐行落 CSV，并汇总 p50/p95/p99。

工作方式
========
1. 用 transient_local + reliable 订阅 ``/map``（等 map server 的 latched 消息），
   用于 ``--count`` 模式在地图范围内按固定 seed 采样目标点；拿不到地图直接报错退出。
2. 向 ``/goal_pose`` 发 geometry_msgs/PoseStamped。
3. 订阅 ``/plan``，以 **header.stamp 变化** 判定「这是新规划结果」（同一条路径被
   重发时 stamp 也会变，所以 stamp 是比「收到一条消息」更准确的判据；个别发布方
   stamp 恒为 0 时退化为「路径长度变化」判定，见 ``run_trial``）。
4. 每次试验记录：目标、重复轮次、延迟、路径长度、ok；超时行延迟/长度留空、
   ok=0。延迟只统计成功样本的 p50/p95/p99。

用法
====
::

    source install/setup.bash
    # 指定若干目标点（map 系，x,y 用分号隔开）
    ros2 run navigation2 bench_planning.py \
        --goals "1.0,2.0;3.5,-1.5;-2.0,0.5" --timeout 2.0 --repeat 3

    # 或者在地图范围内均匀采样 20 个目标（固定 seed，可复现）
    ros2 run navigation2 bench_planning.py --count 20 --seed 7 \
        --csv /tmp/bench_planning.csv

    # 换话题名 / 对准 MINCO 输出
    ros2 run navigation2 bench_planning.py --count 10 \
        --plan-topic /plan --goal-topic /goal_pose

CSV 列：``epoch_s,goal_x,goal_y,repeat,latency_ms,path_length_m,ok``。

注意：本脚本只依赖规划链路的话题（/map、/goal_pose、/plan），不需要仿真器或
Gazebo；但需要上游有节点消费 /goal_pose 并产出 /plan（例如完整导航栈，或
global planner + MINCO smoother）。若 /plan 是周期重发而非按目标触发，拿到的
延迟会包含等待下一拍的开销 —— 这正是 8 Hz 回调抖动要暴露的现象。
"""

import argparse
import csv
import math
import random
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def parse_goals(text):
    """把 ``"x,y;x,y"`` 解析成 [(x, y), ...]。空段跳过，格式错抛 ValueError."""
    goals = []
    for chunk in text.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.split(',')
        if len(parts) != 2:
            raise ValueError(f'bad goal "{chunk}", expected "x,y"')
        try:
            goals.append((float(parts[0]), float(parts[1])))
        except ValueError as exc:
            raise ValueError(f'bad goal "{chunk}": {exc}') from exc
    if not goals:
        raise ValueError('--goals is empty')
    return goals


def path_length_m(path_msg):
    """相邻 pose 的平面欧氏距离之和（米）。MINCO 输出是二维轨迹，z 恒 0."""
    poses = path_msg.poses
    total = 0.0
    for prev, cur in zip(poses, poses[1:]):
        total += math.hypot(
            cur.pose.position.x - prev.pose.position.x,
            cur.pose.position.y - prev.pose.position.y)
    return total


def stamp_key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def percentile(values, p):
    """线性插值百分位（p 取 0-100）。空列表返回 NaN."""
    if not values:
        return float('nan')
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * p / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    frac = rank - lower
    return ordered[lower] * (1.0 - frac) + ordered[upper] * frac


def sample_goals_uniform(map_msg, count, seed):
    """
    Sample goals uniformly inside the map rectangle (axis-aligned map).

    采样点可能落在障碍/未知区 —— 这正是「真实随机目标」的一部分，失败与超时
    同样进入统计；需要精确控制时用 ``--goals`` 指定可通行点。
    """
    info = map_msg.info
    x_min = info.origin.position.x
    y_min = info.origin.position.y
    width = info.width * info.resolution
    height = info.height * info.resolution
    rng = random.Random(seed)
    return [
        (x_min + rng.uniform(0.0, width), y_min + rng.uniform(0.0, height))
        for _ in range(count)
    ]


class BenchPlanning(Node):
    def __init__(self, args):
        super().__init__('bench_planning')
        self.args = args
        self._map_msg = None
        self._plan_count = 0
        self._plan_stamp = None
        self._plan_length = None

        # /map 是 map server 的 latched 消息：transient_local + reliable，迟到也能拿到。
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, map_qos)
        self._plan_sub = self.create_subscription(
            Path, args.plan_topic, self._on_plan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)

    # —— 订阅回调 ——
    def _on_map(self, msg):
        self._map_msg = msg

    def _on_plan(self, msg):
        self._plan_count += 1
        self._plan_stamp = stamp_key(msg)
        self._plan_length = path_length_m(msg)

    # —— 等待/推进 ——
    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(deadline - time.monotonic(), 0.0)))

    def wait_for_map(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and self._map_msg is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._map_msg

    def wait_for_goal_subscribers(self, timeout_s=5.0):
        """等 /goal_pose 有订阅者再开始，避免前几拍目标被丢弃。只告警不致命."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._goal_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().warning(
            f'No subscriber discovered on {self.args.goal_topic} within '
            f'{timeout_s:.1f}s; continuing anyway (goals may be dropped)')
        return False

    # —— 单次试验 ——
    def run_trial(self, goal):
        """
        Send one goal and wait for a new /plan (header.stamp change).

        返回 dict：ok / latency_ms / path_length_m。超时为 ok=False 且后两者 None。
        """
        base_count = self._plan_count
        base_stamp = self._plan_stamp
        base_length = self._plan_length

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.args.frame_id
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(goal[0])
        goal_msg.pose.position.y = float(goal[1])
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        start = time.monotonic()
        self._goal_pub.publish(goal_msg)

        deadline = start + self.args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(deadline - time.monotonic(), 0.0)))
            if self._plan_count <= base_count:
                continue
            new_stamp = self._plan_stamp
            # 主判据：header.stamp 变化。少数发布方 stamp 恒 0，此时用路径内容
            # （长度）变化兜底，否则会把每次试验都记成超时。
            changed = base_stamp is None or new_stamp != base_stamp
            if not changed and new_stamp == (0, 0) and self._plan_length != base_length:
                changed = True
            if not changed:
                continue
            return {
                'ok': True,
                'latency_ms': (time.monotonic() - start) * 1000.0,
                'path_length_m': self._plan_length,
            }

        return {'ok': False, 'latency_ms': None, 'path_length_m': None}


def write_csv(path, rows):
    """按固定列头写 CSV。超时行的 latency_ms/path_length_m 留空、ok=0."""
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ['epoch_s', 'goal_x', 'goal_y', 'repeat', 'latency_ms', 'path_length_m', 'ok'])
        for row in rows:
            writer.writerow([
                f"{row['epoch_s']:.3f}",
                f"{row['goal_x']:.4f}",
                f"{row['goal_y']:.4f}",
                row['repeat'],
                '' if row['latency_ms'] is None else f"{row['latency_ms']:.3f}",
                '' if row['path_length_m'] is None else f"{row['path_length_m']:.4f}",
                1 if row['ok'] else 0,
            ])


def print_summary(rows, csv_path):
    total = len(rows)
    succeeded = [row for row in rows if row['ok']]
    latencies = [row['latency_ms'] for row in succeeded]
    lengths = [row['path_length_m'] for row in succeeded]
    rate = (100.0 * len(succeeded) / total) if total else 0.0

    print('')
    print('== bench_planning summary ==')
    print(f"trials={total} success={len(succeeded)} ({rate:.1f}%) "
          f"timeout={total - len(succeeded)}")
    if latencies:
        print(
            f"latency_ms  p50={percentile(latencies, 50):.2f} "
            f"p95={percentile(latencies, 95):.2f} "
            f"p99={percentile(latencies, 99):.2f} "
            f"mean={sum(latencies) / len(latencies):.2f} "
            f"max={max(latencies):.2f}")
        print(
            f"path_len_m  mean={sum(lengths) / len(lengths):.2f} "
            f"min={min(lengths):.2f} max={max(lengths):.2f}")
    else:
        print('latency_ms  n/a (no successful trial)')
    print(f"CSV: {csv_path} ({total} rows)")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Benchmark end-to-end planning latency via /goal_pose -> /plan.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        '--goals', type=str,
        help='semicolon-separated map-frame goals, e.g. "1.0,2.0;3.5,-1.5"')
    source.add_argument(
        '--count', type=int,
        help='number of goals sampled uniformly inside the /map extent (needs /map)')
    parser.add_argument('--timeout', type=float, default=2.0,
                        help='per-goal wait for a new /plan (seconds)')
    parser.add_argument('--repeat', type=int, default=1,
                        help='repeat the whole goal list N times')
    parser.add_argument('--csv', type=str, default='/tmp/bench_planning.csv',
                        help='CSV output path (written at the end)')
    parser.add_argument('--plan-topic', type=str, default='/plan',
                        help='planned path topic to observe')
    parser.add_argument('--goal-topic', type=str, default='/goal_pose',
                        help='goal pose topic to publish')
    parser.add_argument('--map-topic', type=str, default='/map',
                        help='occupancy grid topic used for --count sampling')
    parser.add_argument('--frame-id', type=str, default='map',
                        help='frame_id of published goals')
    parser.add_argument('--seed', type=int, default=0,
                        help='RNG seed for --count sampling')
    parser.add_argument('--settle', type=float, default=0.2,
                        help='drain time after each trial before the next goal (seconds)')
    parser.add_argument('--map-timeout', type=float, default=5.0,
                        help='wait for the transient_local /map before starting (seconds)')
    args = parser.parse_args(argv)

    if args.count is not None and args.count <= 0:
        parser.error('--count must be > 0')
    if args.timeout <= 0.0:
        parser.error('--timeout must be > 0')
    if args.repeat < 1:
        parser.error('--repeat must be >= 1')
    if args.settle < 0.0:
        parser.error('--settle must be >= 0')
    if args.map_timeout <= 0.0:
        parser.error('--map-timeout must be > 0')
    return args


def main(argv=None):
    args = parse_args(argv)

    goals = None
    if args.goals is not None:
        try:
            goals = parse_goals(args.goals)
        except ValueError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return 2

    rclpy.init(args=[])
    node = BenchPlanning(args)
    rows = []
    exit_code = 0
    try:
        if args.count is not None:
            map_msg = node.wait_for_map(args.map_timeout)
            if map_msg is None:
                node.get_logger().error(
                    f'No transient_local message on {args.map_topic} within '
                    f'{args.map_timeout:.1f}s; cannot sample goals (is the nav stack '
                    f'and map server running?)')
                return 3
            goals = sample_goals_uniform(map_msg, args.count, args.seed)
            node.get_logger().info(
                f'Sampled {len(goals)} goals inside /map '
                f'({map_msg.info.width}x{map_msg.info.height} @ '
                f'{map_msg.info.resolution:.3f} m), seed={args.seed}')

        node.wait_for_goal_subscribers()
        node.spin_for(0.2)  # 让 /plan 订阅先收到最后一条旧消息，建立基线
        node.get_logger().info(
            f'Benchmark: {len(goals)} goal(s) x {args.repeat} repeat(s), '
            f'timeout {args.timeout:.2f}s, plan topic {args.plan_topic}')

        for repeat_index in range(1, args.repeat + 1):
            for goal_index, goal in enumerate(goals, start=1):
                # 上一拍可能还有迟到的 /plan，先排空，避免被算进下一拍延迟。
                node.spin_for(args.settle)
                result = node.run_trial(goal)
                rows.append({
                    'epoch_s': time.time(),
                    'goal_x': goal[0],
                    'goal_y': goal[1],
                    'repeat': repeat_index,
                    'latency_ms': result['latency_ms'],
                    'path_length_m': result['path_length_m'],
                    'ok': result['ok'],
                })
                if result['ok']:
                    node.get_logger().info(
                        f"[{repeat_index}/{args.repeat}] goal {goal_index}/{len(goals)} "
                        f"({goal[0]:.2f},{goal[1]:.2f}) ok "
                        f"latency={result['latency_ms']:.1f}ms "
                        f"len={result['path_length_m']:.2f}m")
                else:
                    node.get_logger().warning(
                        f"[{repeat_index}/{args.repeat}] goal {goal_index}/{len(goals)} "
                        f"({goal[0]:.2f},{goal[1]:.2f}) TIMEOUT after "
                        f"{args.timeout:.2f}s")

        write_csv(args.csv, rows)
        print_summary(rows, args.csv)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted; writing partial CSV')
        write_csv(args.csv, rows)
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
