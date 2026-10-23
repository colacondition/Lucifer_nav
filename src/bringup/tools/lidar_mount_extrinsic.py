#!/usr/bin/env python3
"""lidar_mount_extrinsic.py —— 把 `base_link -> livox_frame`（A 的安装外参）一次算全。

为什么需要它：现有 `lidar_extrinsic_calibration.py` 用"原地打转圆拟合"只能测出
**x、y**（雷达绕底盘旋转中心画圆）；而 `z`、`yaw` 绕轴不可观测只能手填，
**roll/pitch（倾角）**它也不测——打转不改变倾角对圆轨迹的影响。

本脚本把三路来源合成一份完整外参：

    x, y      ← --spin-report（`lidar_extrinsic_calibration.py` 产出的 .json）
    roll,pitch← --imu-* （静止时 A 的 IMU 加速度 = 重力方向，10 s 均值足够）
    z, yaw    ← 手工（--z 卷尺量离地高度；--yaw-deg 车体前向对齐）

输出可直接粘贴的 `measurement_params_real.yaml` 片段，并做几条自检：
  * 倾角 > 5° 时提醒"确认是不是 xacro 默认的 45° 那种安装"；
  * z 与 `segmentation.yaml` 的 `sensor_height` 不一致时告警；
  * IMU 姿态离散度太大时告警（车在动/有人碰）。

用法：
    # ① 先跑原地打转（已有工具），得到 x,y（以及它记下的 z/yaw）
    python3 tools/lidar_extrinsic_calibration.py <spin.bag> \\
        --mount-z 0.49 --mount-yaw-deg 0 --output /tmp/mount_spin

    # ② 合成完整外参（车静止，别碰）
    python3 tools/lidar_mount_extrinsic.py --spin-report /tmp/mount_spin.json \\
        --imu-topic /livox/imu --imu-seconds 10

    # 只差倾角时也可以离线喂一个均值向量
    python3 tools/lidar_mount_extrinsic.py --x 0.02 --y 0.0 --z 0.49 --yaw-deg 0 \\
        --imu-mean "0,0.0872,0.9962"

    # 确认无误后写回配置（自动备份）
    python3 tools/lidar_mount_extrinsic.py --spin-report /tmp/mount_spin.json \\
        --imu-topic /livox/imu --write ../config/reality/measurement_params_real.yaml
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import sys
import time
from pathlib import Path

import numpy as np

DEFAULT_YAML = Path(__file__).resolve().parent.parent / 'config' / 'reality' / \
    'measurement_params_real.yaml'


# --------------------------------------------------------------------------
def accelerometer_to_rp(mean_acc: np.ndarray) -> tuple[float, float, float, str]:
    """静止时的加速度均值 → (roll, pitch, 倾斜模长, 备注)。

    约定：加速度计静止时测到的是"向上的支撑力"，即水平时 ≈ (0,0,+1) g。
    若读到的 z 是负的（某些固件方向相反），自动取反并给出提示。
    """
    vec = np.asarray(mean_acc, dtype=float)
    norm = float(np.linalg.norm(vec))
    if norm < 1e-6:
        raise ValueError('加速度均值几乎为零，IMU 是不是没数据？')
    note = ''
    if vec[2] < 0.0:
        vec = -vec
        note = '加速度计朝下（-1g），已取反'
    unit = vec / norm
    roll = math.atan2(unit[1], unit[2])
    pitch = math.atan2(-unit[0], math.hypot(unit[1], unit[2]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, unit[2]))))
    return roll, pitch, tilt, note


def read_spin_report(path: Path) -> dict:
    data = json.loads(path.read_text())
    tr = data.get('transforms', {}).get('base_link_to_livox_frame')
    if not tr:
        raise KeyError(f'{path} 里没有 transforms.base_link_to_livox_frame')
    cfg = (data.get('configuration', {})
           .get('separated_yaml', {})
           .get('base_link2livox_frame', {}))
    return {
        'x': float(tr['translation_m']['x']),
        'y': float(tr['translation_m']['y']),
        'z': float(tr['translation_m']['z']),
        'yaw_rad': float(tr['rpy_rad']['yaw']),
        'quality': data.get('quality', {}),
        'yaml_snippet': cfg,
    }


def sample_imu(topic: str, seconds: float) -> np.ndarray:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class Sampler(Node):
        def __init__(self) -> None:
            super().__init__('imu_tilt_sampler')
            self.samples: list[np.ndarray] = []
            self.create_subscription(Imu, topic, self.cb, qos_profile_sensor_data)

        def cb(self, msg) -> None:
            a = msg.linear_acceleration
            self.samples.append(np.array([a.x, a.y, a.z], dtype=float))

    rclpy.init()
    node = Sampler()
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if len(node.samples) < 20:
        raise RuntimeError(f'{topic} 只收到 {len(node.samples)} 条 IMU，先确认驱动在跑')
    arr = np.array(node.samples)
    print(f'  IMU 样本 {len(arr)} 条（{topic}），模长均值 {np.linalg.norm(arr, axis=1).mean():.4f} g')
    return arr


def tilt_dispersion(arr: np.ndarray) -> tuple[float, float]:
    """逐样本算 roll/pitch，返回 (std, max-min) 角度（用来判断"有没有被碰"）。"""
    rps = np.array([accelerometer_to_rp(a)[:2] for a in arr[::max(1, len(arr) // 500)]])
    return float(np.degrees(rps.std(axis=0).max())), \
        float(np.degrees(rps.max(axis=0) - rps.min(axis=0)).max())


def patch_yaml(path: Path, xyz: str, rpy: str) -> Path | None:
    text = path.read_text()
    backup = path.with_suffix(path.suffix + f'.bak.{time.strftime("%Y%m%d_%H%M%S")}')
    new = re.sub(r'(\n\s*xyz:\s*).*', lambda m: m.group(1) + f'"\\"{xyz}\\""', text, count=1)
    new = re.sub(r'(\n\s*rpy:\s*).*', lambda m: m.group(1) + f'"\\"{rpy}\\""', new, count=1)
    if new == text:
        return None
    shutil.copy2(path, backup)
    path.write_text(new)
    return backup


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(r: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-8:
        return (math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0]))
    return math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0


def parse_extrinsic(text: str) -> np.ndarray:
    vals = [float(v) for v in text.replace(',', ' ').replace('[', ' ').replace(']', ' ').split()]
    if len(vals) != 6:
        raise ValueError('外参需要 6 个数：x y z roll pitch yaw')
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(*vals[3:])
    t[:3, 3] = vals[:3]
    return t


def read_extrinsic_b_from_yaml(path: Path) -> np.ndarray:
    text = path.read_text()
    m = re.search(r'merge_extrinsic_secondary_to_primary:\s*\n?\s*\[([^\]]+)\]', text)
    if not m:
        raise KeyError(f'{path} 里没找到 merge_extrinsic_secondary_to_primary')
    return parse_extrinsic(m.group(1))


def read_sensor_height() -> float | None:
    seg = DEFAULT_YAML.parent / 'segmentation.yaml'
    if not seg.is_file():
        return None
    m = re.search(r'sensor_height:\s*([0-9.]+)', seg.read_text())
    return float(m.group(1)) if m else None


def main() -> int:
    ap = argparse.ArgumentParser(description='合成 base_link -> livox_frame 完整外参')
    ap.add_argument('--spin-report', type=Path, help='lidar_extrinsic_calibration.py 产出的 .json')
    ap.add_argument('--x', type=float, help='不使用 spin-report 时手填 x（米）')
    ap.add_argument('--y', type=float, help='不使用 spin-report 时手填 y（米）')
    ap.add_argument('--z', type=float, help='离地高度（米）——必须给或从报告里读')
    ap.add_argument('--yaw-deg', type=float, help='车体前向相对雷达前向的偏航（度）')
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--imu-seconds', type=float, default=10.0)
    ap.add_argument('--imu-mean',
                    help='离线：直接给加速度均值，例 --imu-mean=0,0.0872,0.9962。'
                         '值以负号开头时**必须用等号形式**，否则 argparse 会当成选项')
    ap.add_argument('--imu-max-tilt-std-deg', type=float, default=0.5,
                    help='IMU 姿态离散度超过它就告警（车被碰过？）')
    ap.add_argument('--write', type=Path, nargs='?', const=DEFAULT_YAML,
                    help='写回 measurement_params_real.yaml（自动备份）')
    ap.add_argument('--extrinsic-b', dest='extrinsic_b',
                    help='副雷达外参 "x y z roll pitch yaw"（B→A，米/弧度），'
                         '给了就把 base_link->livox_frame_b 一起算出来')
    ap.add_argument('--extrinsic-b-from', type=Path,
                    help='从驱动参数文件里读 merge_extrinsic_secondary_to_primary'
                         '（e.g. config/reality/mid360_driver_dual.yaml）')
    ap.add_argument('--dry-run', action='store_true', help='只打印，不写文件')
    args = ap.parse_args()

    spin = None
    if args.spin_report:
        spin = read_spin_report(args.spin_report)
        print(f'打转标定：{args.spin_report}')
        print(f'  x={spin["x"]:+.4f} m  y={spin["y"]:+.4f} m  '
              f'（yaw={math.degrees(spin["yaw_rad"]):+.2f}° 来自那次的手填值）')
        q = spin['quality']
        if 'circle_fit_rmse_mm' in q:
            print(f'  圆拟合 RMSE {q["circle_fit_rmse_mm"]:.1f} mm，'
                  f'yaw 覆盖 {q.get("yaw_coverage_deg", float("nan")):.0f}°')
    x = args.x if args.x is not None else (spin['x'] if spin else None)
    y = args.y if args.y is not None else (spin['y'] if spin else None)
    if x is None or y is None:
        ap.error('需要 --spin-report 或同时给 --x/--y')
    z = args.z if args.z is not None else (spin['z'] if spin else None)
    if z is None:
        ap.error('必须给 --z（离地高度，绕轴不可观测）')
    yaw_deg = args.yaw_deg if args.yaw_deg is not None else \
        (math.degrees(spin['yaw_rad']) if spin else 0.0)

    # ---- 倾角 ----
    if args.imu_mean:
        vec = np.array([float(v) for v in args.imu_mean.replace(',', ' ').split()])
        print(f'倾角：离线均值 [{vec[0]:+.4f} {vec[1]:+.4f} {vec[2]:+.4f}]')
        disp = None
    else:
        print(f'倾角：采集 {args.imu_seconds:.0f}s 静止 IMU（{args.imu_topic}），别碰车')
        arr = sample_imu(args.imu_topic, args.imu_seconds)
        vec = arr.mean(axis=0)
        disp = tilt_dispersion(arr)
    roll, pitch, tilt, note = accelerometer_to_rp(vec)
    if note:
        print(f'  注意：{note}')

    roll_deg, pitch_deg = math.degrees(roll), math.degrees(pitch)

    # ---- 输出 ----
    xyz = f'{x:.3f} {y:.3f} {z:.3f}'
    rpy = f'{roll:.6f} {pitch:.6f} {math.radians(yaw_deg):.6f}'
    print('\n================ base_link -> livox_frame ================')
    print(f'  xyz : {xyz}            (m)')
    print(f'  rpy : {rpy}')
    print(f'  人读 : x={x:+.3f} y={y:+.3f} z={z:.3f} m | '
          f'roll={roll_deg:+.2f}° pitch={pitch_deg:+.2f}° yaw={yaw_deg:+.1f}° | '
          f'倾斜模长 {tilt:.2f}°')
    print('\n可直接粘进 measurement_params_real.yaml：')
    print(f'  base_link2livox_frame:')
    print(f'    xyz: "\\"{xyz}\\""')
    print(f'    rpy: "\\"{rpy}\\""')
    print('==========================================================')

    # ---- B 的绝对位姿（T_base_B = T_base_A · T_A_B）----
    ext_b = None
    if args.extrinsic_b_from:
        try:
            ext_b = read_extrinsic_b_from_yaml(args.extrinsic_b_from)
            print(f'\n副雷达外参：从 {args.extrinsic_b_from.name} 读到')
        except Exception as exc:
            print(f'  ⚠️  读 {args.extrinsic_b_from} 失败：{exc}')
            ext_b = None
    elif args.extrinsic_b:
        ext_b = parse_extrinsic(args.extrinsic_b)

    if ext_b is not None:
        t_base_a = np.eye(4)
        t_base_a[:3, :3] = rpy_to_matrix(roll, pitch, math.radians(yaw_deg))
        t_base_a[:3, 3] = [x, y, z]
        t_base_b = t_base_a @ ext_b
        bx, by, bz = t_base_b[:3, 3]
        br, bp, byaw = matrix_to_rpy(t_base_b[:3, :3])
        # B 相对车体的倾斜模长（只看 roll/pitch）
        b_tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(br) * math.cos(bp)))))
        bx, by, bz = float(bx), float(by), float(bz)
        print('\n============ base_link -> livox_frame_b（推导值）============')
        print(f'  xyz : {bx:.3f} {by:.3f} {bz:.3f}')
        print(f'  rpy : {br:.6f} {bp:.6f} {byaw:.6f}')
        print(f'  人读 : x={bx:+.3f} y={by:+.3f} z={bz:.3f} m | '
              f'roll={math.degrees(br):+.2f}° pitch={math.degrees(bp):+.2f}° '
              f'yaw={math.degrees(byaw):+.1f}° | 相对车体倾斜 {b_tilt:.2f}°')
        print('  （这条只用于可视化/排障；合并链路不需要它）')
        print('==========================================================')

    # ---- 自检 ----
    warn = 0
    if disp is not None and disp[0] > args.imu_max_tilt_std_deg:
        print(f'  ⚠️  IMU 姿态离散度 {disp[0]:.2f}°（> {args.imu_max_tilt_std_deg}°），'
              f'车可能被碰过，建议重测')
        warn += 1
    if tilt > 5.0:
        print(f'  ⚠️  倾斜 {tilt:.2f}° 不小 —— 确认是否就是 xacro 默认的 45° 那种安装；'
              f'倾斜后 /scan 与代价地图高度带都依赖这条 TF')
        warn += 1
    height = read_sensor_height()
    if height is not None and abs(height - z) > 0.02:
        print(f'  ⚠️  z={z:.3f} 与 segmentation.yaml 的 sensor_height={height:.3f} 差 '
              f'{abs(height - z) * 100:.1f} cm —— 两者必须一致')
        warn += 1
    if abs(x) + abs(y) > 0.6:
        print(f'  ⚠️  水平偏移 {math.hypot(x, y) * 100:.1f} cm 偏大，检查 base_link 原点'
              f'是不是底盘几何中心（原地打转的旋转中心）')
        warn += 1
    if warn == 0:
        print('  ✅ 自检通过')

    # ---- 写回 ----
    if args.write and not args.dry_run:
        backup = patch_yaml(args.write, xyz, rpy)
        if backup:
            print(f'\n✅ 已写回 {args.write}（备份 {backup.name}）')
        else:
            print(f'\n⚠️  {args.write} 里没找到 xyz/rpy，未修改')
    elif args.write:
        print('\n（--dry-run：未写文件）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
